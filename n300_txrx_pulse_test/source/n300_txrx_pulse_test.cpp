//
// Copyright 2010-2011 Ettus Research LLC
// Copyright 2018 Ettus Research, a National Instruments Company
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/device3.hpp>
#include <uhd/rfnoc/radio_ctrl.hpp>
#include <uhd/utils/thread.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <complex>
#include <iostream>

#define USE_MULTI_USRP 0

namespace po = boost::program_options;

typedef struct {
  int rx0;
  int rx1;
  int tx0;
  int tx1;
} ch_select_t;

uhd::rfnoc::radio_ctrl::sptr _radio_ctrl;
uhd::device3::sptr _usrp;
uhd::usrp::multi_usrp::sptr _multiusrp;
uhd::rx_streamer::sptr _rx_stream;
uhd::rx_streamer::sptr _rx_cal_stream;
uhd::tx_streamer::sptr _tx_stream;
uhd::tx_streamer::sptr _tx_cal_stream;

template<typename data_type> void file2wave(std::vector<std::complex<data_type>> &data, const std::string &fname){
    // read complex data files with extension .dat
    boost::filesystem::path p(fname);
    std::ifstream ifile;
    ifile.open(fname.c_str(), std::ios::binary);
    std::vector<uint32_t> datavec;
    if (ifile.is_open())
    {
        ifile.seekg(0,ifile.end);
        int flen = ifile.tellg();
        ifile.seekg(0,ifile.beg);
        datavec.resize(flen/sizeof(uint32_t));
        data.reserve(datavec.size());
        ifile.read((char*)&datavec.front(), datavec.size()*sizeof(uint32_t));
        ifile.close();
         for(size_t i=0; i< datavec.size() ;i++){
            int16_t tempU,tempL;
            std::complex<data_type> temp;
            tempL = (int16_t)(datavec[i] & 0x0000FFFF);
            tempU = (int16_t)((datavec[i] & 0xFFFF0000)>>16);
            if ((p.extension().string()==".dat") or (p.extension().string()==".ref"))
                temp = std::complex<data_type>((data_type)tempL,(data_type)tempU);
            else
                temp = std::complex<data_type>((data_type)tempU,(data_type)tempL);
             data.push_back(temp);
         }
        return;
    }
    throw(std::runtime_error("Could not open file"));
}

int sync_pps(double &time_set,double time_req){
    double rate = _radio_ctrl->get_rate();
    std::string timesrc = _radio_ctrl->get_time_source();
    // time_req ignored for gpsdo
    time_set = 0.0;
    if (timesrc == "gpsdo"){
        uhd::property_tree::sptr tree = _usrp->get_tree();
        uhd::fs_path path;
        std::vector<std::string> mboard_names = tree->list("/mboards");
        path = "/mboards/" + mboard_names[0];

        std::vector<std::string> sensor_names = tree->list(path / "sensors");
        //Check for 10 MHz lock
        int gps_time=0;
        if(std::find(sensor_names.begin(), sensor_names.end(), "gps_time") != sensor_names.end()) {
            gps_time = tree->access<uhd::sensor_value_t>(path / "sensors" / "gps_time").get().to_int();
        }
        else if(std::find(sensor_names.begin(), sensor_names.end(), "get_gps_time_sensor") != sensor_names.end()) {
            try{
                gps_time = tree->access<uhd::sensor_value_t>(path / "sensors" / "get_gps_time_sensor").get().to_int();
            }
            catch (std::exception &e) {
                std::cout<<"Error caught exception while accessing get_gps_time_sensor: "<<e.what()<<std::endl;
                return -1;
            }
        }
        else{
            std::cerr<< "[usrp_controller::sync_pps] Error: gps_time sensor field not found"<<std::endl;
            return -1;
        }
        uint64_t last_pps = _radio_ctrl->get_time_last_pps().to_ticks(rate);
        uint64_t curr_pps = last_pps;
        while (curr_pps ==last_pps){
            curr_pps = _radio_ctrl->get_time_last_pps().to_ticks(rate);
            boost::this_thread::sleep(boost::posix_time::milliseconds(20));
        }
        double gps_time_next_d = (double)gps_time+2.0;
        uhd::time_spec_t new_gps_time = uhd::time_spec_t(gps_time_next_d);
        _radio_ctrl->set_time_next_pps(new_gps_time);
        time_set = gps_time_next_d;
        return(0);
    }
    else{
        uint64_t last_pps = _radio_ctrl->get_time_last_pps().to_ticks(rate);
        uint64_t curr_pps = last_pps;
        while (curr_pps ==last_pps){
            curr_pps = _radio_ctrl->get_time_last_pps().to_ticks(rate);
            boost::this_thread::sleep(boost::posix_time::milliseconds(20));
        }
        if (time_req >=0.0){
          _radio_ctrl->set_time_next_pps(uhd::time_spec_t(time_req));
          time_set = time_req;
      }
      else{
        double time_set_next = _radio_ctrl->get_time_last_pps().get_real_secs()+1.0;
        _radio_ctrl->set_time_next_pps(uhd::time_spec_t(time_set_next));
        time_set = time_set_next;
      }
        return(0);
    }
}

void pretty_print_flow_graph(std::vector<std::string> blocks)  {
    std::string sep_str = "==>";
    std::cout << std::endl;
    // Line 1
    for (size_t n = 0; n < blocks.size(); n++) {
        const std::string name = blocks[n];
        std::cout << "+";
        for (size_t i = 0; i < name.size() + 2; i++) {
            std::cout << "-";
        }
        std::cout << "+";
        if (n == blocks.size() - 1) {
            break;
        }
        for (size_t i = 0; i < sep_str.size(); i++) {
            std::cout << " ";
        }
    }
    std::cout << std::endl;
    // Line 2
    for (size_t n = 0; n < blocks.size(); n++) {
        const std::string name = blocks[n];
        std::cout << "| " << name << " |";
        if (n == blocks.size() - 1) {
            break;
        }
        std::cout << sep_str;
    }
    std::cout << std::endl;
    // Line 3
    for (size_t n = 0; n < blocks.size(); n++) {
        const std::string name = blocks[n];
        std::cout << "+";
        for (size_t i = 0; i < name.size() + 2; i++) {
            std::cout << "-";
        }
        std::cout << "+";
        if (n == blocks.size() - 1) {
            break;
        }
        for (size_t i = 0; i < sep_str.size(); i++) {
            std::cout << " ";
        }
    }
    std::cout << std::endl << std::endl;
}

void pulseStream(std::vector<std::complex<short>> &pulseVector,std::vector<uhd::rx_metadata_t> & md_vec, unsigned long num_rx ,double seconds_in_future, double timestart, std::string &current_wavefile, ch_select_t ch_select){

    std::vector<std::complex<short>> data;
    file2wave<short>(data,current_wavefile);
    if (data.size()>num_rx){
      std::cout<<"WARNING: TX waveform is longer ("<<data.size()<<" samples) than requested RX nsamps ("<<num_rx<<")"<<std::endl;
    }
    // std::vector<std::complex<short>> buff(data.size()+num_rx_extra);
    std::vector<std::complex<short>> buff(num_rx);

    uhd::time_spec_t timenow;
    if (timestart < 0.0){
      timenow =  _radio_ctrl->get_time_now();
    }
    else{
        timenow = uhd::time_spec_t(timestart);
    }
    uhd::time_spec_t time_spec = uhd::time_spec_t(seconds_in_future)+timenow;

    //setup metadata for the first packet
    uhd::tx_metadata_t md_tx;
    md_tx.start_of_burst = true;
    md_tx.end_of_burst = true;
    md_tx.has_time_spec = true;
    md_tx.time_spec = time_spec;

    //setup streaming
    uhd::rx_metadata_t md_rx;
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps = buff.size();
    stream_cmd.time_spec = time_spec;
    stream_cmd.stream_now = false;

    if (ch_select.tx0==1)
      _tx_stream->send(&data.front(), data.size(), md_tx);
    else if (ch_select.tx1==1)
      _tx_cal_stream->send(&data.front(), data.size(), md_tx);

    size_t num_rx_samps;
    double rx_timeout = 3.0;
    if (ch_select.rx0==1){
        _rx_stream ->issue_stream_cmd(stream_cmd);
        num_rx_samps = _rx_stream->recv(&buff.front(), buff.size(), md_rx, rx_timeout);
      }
    else if (ch_select.rx1==1){
        _rx_cal_stream ->issue_stream_cmd(stream_cmd);
        num_rx_samps = _rx_cal_stream->recv(&buff.front(), buff.size(), md_rx, rx_timeout);
     }
     else {
       num_rx_samps = 0;
       return;
     }
    pulseVector.insert(pulseVector.end(),buff.begin(),buff.begin()+num_rx_samps);

    md_vec.push_back(md_rx);
}

int usrpInit(const std::string & inargs,const std::string &timesource, double rate, double freq, double rxgain, double txgain) {
    uhd::set_thread_priority_safe();
    std::string format = "sc16";
    //std::string args = "fpga=/usr/share/uhd/images/usrp_e310_fpga_rfnoc.bit";
    std::string args = inargs; // "skip_sram" //"send_buff_size=131072,max_send_window=32";

    //std::string args = "type=x300,dboard_clock_rate=20e6";
    //std::string args = "type=x300,dboard_clock_rate=50e6";

    std::string radio_args;
    std::string streamargs = "";
    size_t radio_id = 0;
    size_t radio_chan = 0;
    size_t calib_chan = 1;

    double setup_time = 1.0;

     std::string tx_blockid1, rx_blockid1, rx_blockid2;

     tx_blockid1 = "";
     rx_blockid1 = "DmaFIFO";//"FIFO_0";//"FIFO_0";//"DmaFIFO";
     rx_blockid2 = "";//"FIFO_1"

     // Check settings
     if (not tx_blockid1.empty()) {
         if (not uhd::rfnoc::block_id_t::is_valid_block_id(tx_blockid1)) {
             std::cout << "Invalid block ID for the TX processing block." << std::endl;
             return ~0;
         }
     }
     if (not rx_blockid1.empty()) {
         if (not uhd::rfnoc::block_id_t::is_valid_block_id(rx_blockid1)) {
             std::cout << "Invalid block ID for the 1st RX processing block." << std::endl;
             return ~0;
         }
     }
     if (not rx_blockid2.empty()) {
         if (not uhd::rfnoc::block_id_t::is_valid_block_id(rx_blockid2)) {
             std::cout << "Invalid block ID for the 2nd RX processing block." << std::endl;
             return ~0;
         }
     }


    //  uhd::usrp::multi_usrp::sptr multiusrp;
     std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
     try {
        if (USE_MULTI_USRP){
          _multiusrp = uhd::usrp::multi_usrp::make(args);
          _usrp = _multiusrp->get_device3();
        }
        else{
          _usrp = uhd::device3::make(args);
        }
     }
     catch(const std::exception &e)
     {
         std::cout << boost::format("device3::failed to make device %s") % e.what() << std::endl;
         return EXIT_FAILURE;
     }

     uhd::rfnoc::block_id_t radio_ctrl_id(0, "Radio", radio_id);
    _radio_ctrl = _usrp->get_block_ctrl< uhd::rfnoc::radio_ctrl >(radio_ctrl_id);

    // ########################################
    // Set up radio
     // ########################################
    _radio_ctrl->set_args(radio_args);

    //set the sample rate
    if (rate <= 0.0){
        std::cerr << "Please specify a valid sample rate" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << boost::format("Setting RX Rate: %f Msps...") % (rate/1e6) << std::endl;
    _radio_ctrl->set_rate(rate);
    std::cout << boost::format("Actual RX Rate: %f Msps...") % (_radio_ctrl->get_rate()/1e6) << std::endl << std::endl;


    uhd::property_tree::sptr tree = _usrp->get_tree();
    uhd::fs_path path;

    std::string device_name = tree->access<std::string>("/name").get();
    std::cout << "Device: " << device_name << std::endl;

    std::vector<std::string> mboard_names = tree->list("/mboards");

    for(auto name : mboard_names){
        path =  "/mboards/" + name;
        std::cout <<"Sensors: "<<std::endl;
        if (tree->exists(path / "sensors")){
          size_t count = 0;
          std::vector<std::string> prop_names = tree->list(path / "sensors");
          for(const std::string &prop_name:  prop_names){
              try{
                  std::cout << prop_name<<": "<< tree->access<uhd::sensor_value_t>(path / "sensors" / prop_name).get().value << "\n";
              }
              catch (std::exception &e) {
                  std::cout<<"Error caught exception while accessing sensor "<<prop_name<<": "<<e.what()<<std::endl;
              }
          }
      }
    }
    path = "/mboards/" + mboard_names[0];


    std::vector<std::string> sensor_names = tree->list(path / "sensors");


    //Check for 10 MHz lock
    if(std::find(sensor_names.begin(), sensor_names.end(), "ref_locked") != sensor_names.end()) {
        try{
            uhd::sensor_value_t ref_locked = tree->access<uhd::sensor_value_t>(path / "sensors" / "ref_locked").get();
            for (size_t i = 0; not ref_locked.to_bool() and i < 100; i++) {
                boost::this_thread::sleep(boost::posix_time::milliseconds(100));
                ref_locked = tree->access<uhd::sensor_value_t>(path / "sensors" / "ref_locked").get();
            }
            if(not ref_locked.to_bool()) {
                std::cout << boost::format("USRP NOT Locked to GPSDO 10 MHz Reference.\n");
                std::cout << boost::format("Double check installation instructions (N2X0/E1X0 only): https://www.ettus.com/content/files/gpsdo-kit_4.pdf\n\n");
                return EXIT_FAILURE;
            } else {
                std::cout << boost::format("USRP Locked to 10 MHz Reference.\n");
            }
        }
        catch (std::exception &e) {
            std::cout<<"Error caught exception while checking ref locked sensor: "<<e.what()<<std::endl;
        }
    } else {
        std::cout << boost::format("ref_locked sensor not present on this board.\n");
    }

    std::vector<std::string> timesrcs = tree->access<std::vector<std::string>>(path / "time_source" / "options").get();
    std::cout<<"Time Sources:"<<std::endl;

    for (auto i : timesrcs){
      std::cout<<i<<std::endl;
    }
    std::cout<<std::endl;

    std::vector<std::string> time_srcs = _radio_ctrl->get_time_sources();
    std::cout<<"Radio Time Sources:\n";
    for (auto i : time_srcs){
      std::cout<<"\t"<<i<<"\n";
    }
    std::vector<std::string> clk_srcs = _radio_ctrl->get_clock_sources();
    std::cout<<"Radio Clock Sources:\n";
    for (auto i : clk_srcs)
      std::cout<<"\t"<<i<<"\n";


    std::vector<std::string> dboard_names = tree->list(path / "dboards");
    for (auto i : dboard_names){
        std::cout<<"Daughter board " <<i<<" frontends: "<<std::endl;
        uhd::fs_path dpath = path + "/dboards/" + i;
        std::vector<std::string> rx_frontend_names = tree->list(dpath / "rx_frontends");
        for (auto j : rx_frontend_names){
            std::cout<<j<<std::endl;
            std::vector<std::string> rxantennas = tree->access<std::vector<std::string>>(dpath / "rx_frontends" / j / "antenna/options").get();
            std::cout<<"\tRX Antennas"<<std::endl;
            for (auto jj : rxantennas)
                std::cout<<"\t"<<jj<<std::endl;
            std::list<std::string> agcmodes;
            try {
                agcmodes = tree->access<std::list<std::string>>(dpath / "rx_frontends" / j / "gain/agc/mode/options").get();
                std::cout<<"\tRX AGC Modes"<<std::endl;
                for (auto jj : agcmodes)
                    std::cout<<"\t"<<jj<<std::endl;
                std::cout<<"Setting AGC Mode to fast"<<std::endl;
                tree->access<std::string>(dpath / "rx_frontends" / j / "gain/agc/mode/value").set("fast");
            } catch (std::exception &) {}
        }
        std::vector<std::string> tx_frontend_names = tree->list(dpath / "tx_frontends");
        for (auto j : tx_frontend_names){
            std::cout<<j<<std::endl;
            std::vector<std::string> txantennas = tree->access<std::vector<std::string>>(dpath / "tx_frontends" / j / "antenna/options").get();
            std::cout<<"\tTX Antennas"<<std::endl;
            for (auto jj : txantennas)
                std::cout<<"\t"<<jj<<std::endl;
        }

    }

    // Display the LO names and frequency ranges
    std::vector<std::string> lo_names = _radio_ctrl->get_rx_lo_names(radio_chan);
    std::cout<<"LO Names\n";
    for (auto i : lo_names){
      std::cout<<i<<":\n";
      std::vector<std::string> lo_srcs = _radio_ctrl->get_rx_lo_sources(i,radio_chan);
      uhd::freq_range_t fr_rng = _radio_ctrl->get_rx_lo_freq_range(i,radio_chan);
      for (auto j : lo_srcs){
        std::cout<<"\t"<<j<<"\n";
      }
      std::cout<<"\tFreq range: "<<fr_rng.start()<<" - "<<fr_rng.stop()<<"\n";
    }

    std::vector<std::string> cal_lo_names = _radio_ctrl->get_rx_lo_names(calib_chan);
    std::cout<<"Calib CH. LO Names\n";
    for (auto i : cal_lo_names){
      std::cout<<i<<":\n";
      std::vector<std::string> lo_srcs = _radio_ctrl->get_rx_lo_sources(i,calib_chan);
      uhd::freq_range_t fr_rng = _radio_ctrl->get_rx_lo_freq_range(i,calib_chan);

      for (auto j : lo_srcs){
        std::cout<<"\t"<<j<<"\n";
      }
      std::cout<<"\tCalib CH. Freq range: "<<fr_rng.start()<<" - "<<fr_rng.stop()<<"\n";
    }

    std::cout << std::endl << "Time source is first set to " << _radio_ctrl->get_time_source() << std::endl;
    std::cout << std::endl << "Attempting to set time source to gpsdo" << std::endl;

//    Explicitly set time source to gpsdo
    try {
        _radio_ctrl->set_time_source("gpsdo");
    } catch (uhd::key_error &e) {
        std::cout << "could not set the time source to \"gpsdo\"; error was:" <<e.what()<<std::endl;
        std::cout << e.what() << std::endl;
        std::cout << "trying \"external\"..." <<std::endl;
        try {
            _radio_ctrl->set_time_source("external");
        } catch (uhd::key_error &er) {
            std::cout << "\"external\" failed, too. Error: " <<er.what() << std::endl;
        }
        catch (std::exception &e) {
            std::cout<<"\nError caught exception while attempting to set time src external: "<<e.what()<<std::endl;
        }
    }
    catch (std::exception &e) {
        std::cout<<"\nError caught exception while attempting to set time src gpsdo: "<<e.what()<<std::endl;
    }
    std::cout << std::endl << "Time source is now " << _radio_ctrl->get_time_source() << std::endl;

    boost::this_thread::sleep(boost::posix_time::seconds(setup_time));

    // The TCXO has a long warm up time, so wait up to 30 seconds for sensor data to show up
    std::cout << "Waiting for the GPSDO to warm up..." << std::endl;
    for (size_t i = 0; i < 300; i++) {
        try {
            tree->access<uhd::sensor_value_t>(path / "sensors" / "gps_locked").get().value;
            break;
        } catch (std::exception &) {}
        boost::this_thread::sleep(boost::posix_time::milliseconds(100));
    }
    try {
        tree->access<uhd::sensor_value_t>(path / "sensors" / "gps_locked").get().value;
    } catch (std::exception &) {
        std::cout << "No response from GPSDO in 30 seconds" << std::endl;
        return EXIT_FAILURE;
    }
    std::cout << "The GPSDO is warmed up and talking." << std::endl;

    //Check for GPS lock
    uhd::sensor_value_t gps_locked = tree->access<uhd::sensor_value_t>(path / "sensors" / "gps_locked").get();
    if(not gps_locked.to_bool()) {
        std::cout << boost::format("\nGPS does not have lock. Wait a few minutes and try again.\n");
        std::cout << boost::format("NMEA strings and device time may not be accurate until lock is achieved.\n\n");
    } else {
        std::cout << boost::format("GPS Locked");
    }

    if(not timesource.empty()){
    //    Explicitly set time source to gpsdo
        try {
            _radio_ctrl->set_time_source(timesource);
        } catch (uhd::key_error &e) {
            std::cout << "could not set the time source to \""<<timesource<<"\"; error was:" <<e.what()<<std::endl;
        }
        catch (std::exception &e) {
            std::cout<<"\nError std::exception caught exception while attempting to set time src to "<<timesource<<" : "<<e.what()<<std::endl;
        }
        std::cout << std::endl << "Time source is now " << _radio_ctrl->get_time_source() << std::endl;

        boost::this_thread::sleep(boost::posix_time::seconds(setup_time));
    }


    std::cout<<"Checking if Ref is locked... " <<std::endl;
    if(std::find(sensor_names.begin(), sensor_names.end(), "ref_locked") != sensor_names.end()) {
        try{
            uhd::sensor_value_t ref_locked = tree->access<uhd::sensor_value_t>(path / "sensors" / "ref_locked").get();
          if(not ref_locked.to_bool()) {
              std::cout << "\nRef does not have lock. Wait a few minutes and try again."<<std::endl;
          } else {
              std::cout << "Ref Locked"<<std::endl;;
          }
      }
      catch (std::exception &e) {
          std::cout<<"Error caught exception checking ref locked: "<<": "<<e.what()<<std::endl;
      }
    }


    std::cout<<"Time source: "<<_radio_ctrl->get_time_source()<<". Clock source: "<<_radio_ctrl->get_clock_source()<<std::endl<<std::endl;


    //set the center frequency
    std::cout << boost::format("Setting RX Freq: %f MHz...") % (freq/1e6) << std::endl;
    uhd::tune_request_t tune_request(freq);

    //    if (vm.count("int-n")) {
    //        //tune_request.args = uhd::device_addr_t("mode_n=integer"); TODO
    //    }
    _radio_ctrl->set_rx_frequency(freq, radio_chan);
    std::cout << boost::format("Actual RX Freq: %f MHz...") % (_radio_ctrl->get_rx_frequency(radio_chan)/1e6) << std::endl << std::endl;
    _radio_ctrl->set_tx_frequency(freq, radio_chan);
    std::cout << boost::format("Actual TX Freq: %f MHz...") % (_radio_ctrl->get_tx_frequency(radio_chan)/1e6) << std::endl << std::endl;

    _radio_ctrl->set_rx_frequency(freq, calib_chan);
    std::cout << boost::format("Actual Calib CH. RX Freq: %f MHz...") % (_radio_ctrl->get_rx_frequency(calib_chan)/1e6) << std::endl << std::endl;
    _radio_ctrl->set_tx_frequency(freq, calib_chan);
    std::cout << boost::format("Actual Calib CH. TX Freq: %f MHz...") % (_radio_ctrl->get_tx_frequency(calib_chan)/1e6) << std::endl << std::endl;


    //set the rf gain
    std::cout << boost::format("Setting RX Gain: %f dB...") % rxgain << std::endl;
    _radio_ctrl->set_rx_gain(rxgain, radio_chan);
    std::cout << boost::format("Actual RX Gain: %f dB...") % _radio_ctrl->get_rx_gain(radio_chan) << std::endl << std::endl;

    //set the rf gain
    std::cout << boost::format("Setting TX Gain: %f dB...") % txgain << std::endl;
    _radio_ctrl->set_tx_gain(txgain, radio_chan);
    std::cout << boost::format("Actual TX Gain: %f dB...") % _radio_ctrl->get_tx_gain(radio_chan) << std::endl << std::endl;

    std::cout << boost::format("Setting Calib Ch. RX Gain: %f dB...") % rxgain << std::endl;
    _radio_ctrl->set_rx_gain(rxgain, calib_chan);
    std::cout << boost::format("Actual Calib Ch. RX Gain: %f dB...") % _radio_ctrl->get_rx_gain(calib_chan) << std::endl << std::endl;

    //set the rf gain
    std::cout << boost::format("Setting Calib Ch. TX Gain: %f dB...") % txgain << std::endl;
    _radio_ctrl->set_tx_gain(0.0, calib_chan);
    std::cout << boost::format("Actual Calib Ch. TX Gain: %f dB...") % _radio_ctrl->get_tx_gain(calib_chan) << std::endl << std::endl;

     std::string antRX("RX2");
     std::cout << boost::format("Setting RX Antenna: %s") % antRX << std::endl;
     _radio_ctrl->set_rx_antenna(antRX,radio_chan);
     std::cout << boost::format("Actual RX Antenna: %s") % _radio_ctrl->get_rx_antenna(radio_chan) << std::endl << std::endl;
    //
     std::string antTX("TX/RX");
     std::cout << boost::format("Setting TX Antenna: %s") % antTX << std::endl;
     _radio_ctrl->set_tx_antenna(antTX,radio_chan);
     std::cout << boost::format("Actual TX Antenna: %s") % _radio_ctrl->get_tx_antenna(radio_chan) << std::endl << std::endl;

     std::cout << boost::format("Setting Calib Ch. RX Antenna: %s") % antRX << std::endl;
     _radio_ctrl->set_rx_antenna(antRX,calib_chan);
     std::cout << boost::format("Actual Calib Ch. RX Antenna: %s") % _radio_ctrl->get_rx_antenna(calib_chan) << std::endl << std::endl;
    //
     std::cout << boost::format("Setting Calib Ch. TX Antenna: %s") % antTX << std::endl;
     _radio_ctrl->set_tx_antenna(antTX,calib_chan);
     std::cout << boost::format("Actual Calib Ch. TX Antenna: %s") % _radio_ctrl->get_tx_antenna(calib_chan) << std::endl << std::endl;

     double rx_bw = _radio_ctrl->get_rx_bandwidth(radio_chan); // const ;
     std::cout<<"RX BW: "<<rx_bw<<"\n";

     double cal_rx_bw = _radio_ctrl->get_rx_bandwidth(calib_chan); // const;
     std::cout<<"Calib CH. RX BW: "<<cal_rx_bw<<"\n";

     _radio_ctrl->set_tx_bandwidth(rx_bw, radio_chan);
     double tx_bw = _radio_ctrl->get_tx_bandwidth(radio_chan); // const ;
     std::cout<<"TX BW: "<<tx_bw<<"\n";

     _radio_ctrl->set_tx_bandwidth(cal_rx_bw, calib_chan);
     double cal_tx_bw = _radio_ctrl->get_tx_bandwidth(calib_chan); // const;
     std::cout<<"Calib CH. TX BW: "<<cal_tx_bw<<"\n";

    boost::this_thread::sleep(boost::posix_time::seconds(setup_time)); //allow for some setup time


    size_t spp = _radio_ctrl->get_arg<int>("spp");

    // ########################################
     // Set up streaming
    // ########################################

    uhd::device_addr_t rx_streamer_args(streamargs);
    uhd::device_addr_t tx_streamer_args(streamargs);

    uhd::device_addr_t tx_cal_streamer_args(streamargs);
    uhd::device_addr_t rx_cal_streamer_args(streamargs);

    // Reset device streaming state
    _usrp->clear();
    uhd::rfnoc::graph::sptr rx_graph = _usrp->create_graph("rx_graph");
    uhd::rfnoc::graph::sptr tx_graph = _usrp->create_graph("tx_graph");

    /////////////////////////////////////////////////////////////////////////
    //////// 2. Get block control objects ///////////////////////////////////
    /////////////////////////////////////////////////////////////////////////
    std::vector<std::string> blocks;

    // For the processing blocks, we don't care what type the block is,
    // so we make it a block_ctrl_base (default):
    uhd::rfnoc::block_ctrl_base::sptr tx_proc_block_ctrl1, rx_proc_block_ctrl1, rx_proc_block_ctrl2;

    if (not tx_blockid1.empty() and _usrp->has_block(tx_blockid1)) {
        tx_proc_block_ctrl1 = _usrp->get_block_ctrl(tx_blockid1);
        blocks.push_back(tx_proc_block_ctrl1->get_block_id());
    }

    blocks.push_back(_radio_ctrl->get_block_id());

    if (not rx_blockid1.empty() and _usrp->has_block(rx_blockid1)) {
        rx_proc_block_ctrl1 = _usrp->get_block_ctrl(rx_blockid1);
        blocks.push_back(rx_proc_block_ctrl1->get_block_id());
    }
    if (not rx_blockid2.empty() and _usrp->has_block(rx_blockid2)) {
        rx_proc_block_ctrl2 = _usrp->get_block_ctrl(rx_blockid2);
        blocks.push_back(rx_proc_block_ctrl2->get_block_id());
    }

    blocks.push_back("HOST");
    pretty_print_flow_graph(blocks);


    /////////////////////////////////////////////////////////////////////////
    //////// 3. Set channel definitions /////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////
    //
    // Here, we define that there is only 1 channel, and it points
    // to the final processing block.
   if (rx_proc_block_ctrl2 and rx_proc_block_ctrl1) {
        rx_streamer_args["block_id"] = rx_blockid2;
        spp = rx_proc_block_ctrl2->get_args().cast<size_t>("spp", spp);
    } else if (rx_proc_block_ctrl1) {
        rx_streamer_args["block_id"] = rx_blockid1;
        spp = rx_proc_block_ctrl1->get_args().cast<size_t>("spp", spp);
    } else {
        rx_streamer_args["block_id"] = radio_ctrl_id.to_string();
        rx_streamer_args["block_port"] = str(boost::format("%d") % radio_chan);
    }


     if (tx_proc_block_ctrl1) {
         tx_streamer_args["block_id"] = tx_blockid1;
         spp = tx_proc_block_ctrl1->get_args().cast<size_t>("spp", spp);
     } else {
         tx_streamer_args["block_id"] = radio_ctrl_id.to_string();
         tx_streamer_args["block_port"] = str(boost::format("%d") % radio_chan);
     }

    tx_cal_streamer_args["block_id"] = radio_ctrl_id.to_string();
    tx_cal_streamer_args["block_port"] = str(boost::format("%d") % calib_chan);


    rx_cal_streamer_args["block_id"] = radio_ctrl_id.to_string();
    rx_cal_streamer_args["block_port"] = str(boost::format("%d") % calib_chan);


    /////////////////////////////////////////////////////////////////////////
    //////// 5. Connect blocks //////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////
    std::cout << "Connecting blocks..." << std::endl;
    if (tx_proc_block_ctrl1) {
        tx_graph->connect( // Yes, it's that easy!
                tx_proc_block_ctrl1->get_block_id(),uhd::rfnoc::ANY_PORT, radio_ctrl_id,radio_chan
        );
    }

    if (rx_proc_block_ctrl1) {
        rx_graph->connect(
            radio_ctrl_id, radio_chan, rx_proc_block_ctrl1->get_block_id(), uhd::rfnoc::ANY_PORT
        );
    }
    if (rx_proc_block_ctrl2 and rx_proc_block_ctrl1) {
        rx_graph->connect(
            rx_proc_block_ctrl1->get_block_id(),
            rx_proc_block_ctrl2->get_block_id()
        );
    }

    /////////////////////////////////////////////////////////////////////////
    //////// 6. Spawn receiver //////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////
    UHD_LOGGER_INFO("RFNOC") << "Samples per packet: " << spp;
    uhd::stream_args_t rx_stream_args(format, "sc16");
    rx_stream_args.args = rx_streamer_args;
    rx_stream_args.args["spp"] = boost::lexical_cast<std::string>(spp);
    UHD_LOGGER_INFO("RFNOC") << "Using RX streamer args: " << rx_stream_args.args.to_string();

    _rx_stream = _usrp->get_rx_stream(rx_stream_args);

    uhd::stream_args_t rx_cal_stream_args(format, "sc16");

    rx_cal_stream_args.args = rx_cal_streamer_args;
    rx_cal_stream_args.args["spp"] = boost::lexical_cast<std::string>(spp);
    UHD_LOGGER_INFO("RFNOC") << "Using RX Cal streamer args: " << rx_cal_stream_args.args.to_string();

    _rx_cal_stream = _usrp->get_rx_stream(rx_cal_stream_args);


    uhd::stream_args_t tx_stream_args(format, "sc16");
    tx_stream_args.args = tx_streamer_args;
    tx_stream_args.args["spp"] = boost::lexical_cast<std::string>(spp);
    UHD_LOGGER_INFO("RFNOC") << "Using TX streamer args: " << tx_stream_args.args.to_string();

    _tx_stream = _usrp->get_tx_stream(tx_stream_args);

    uhd::stream_args_t tx_cal_stream_args(format, "sc16");
    tx_cal_stream_args.args = tx_cal_streamer_args;
    tx_cal_stream_args.args["spp"] = boost::lexical_cast<std::string>(spp);
    UHD_LOGGER_INFO("RFNOC") << "Using TX CAL streamer args: " << tx_cal_stream_args.args.to_string();

    _tx_cal_stream = _usrp->get_tx_stream(tx_cal_stream_args);

    // set time to now just in case pps never comes
    // set time to zero at next pps
    _radio_ctrl->set_time_now(uhd::time_spec_t(0.0));

    boost::this_thread::sleep(boost::posix_time::seconds(setup_time)); //allow for some setup time


    return EXIT_SUCCESS;
}

int UHD_SAFE_MAIN(int argc, char* argv[])
{
    uhd::set_thread_priority_safe();

    // variables to be set by po
    std::string args,timesrc;
    std::string wire;
    double seconds_in_future;
    size_t total_num_samps, npulses;
    double rate,freq,txgain,rxgain;
    int ch_tx, ch_rx;
    std::string current_wavefile, fname;
    bool syncpps;

    // setup the program options
    po::options_description desc("Allowed options");
    // clang-format off
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "single uhd device address args")
        ("timesrc", po::value<std::string>(&timesrc)->default_value(""), "single uhd device address args")
        ("freq", po::value<double>(&freq)->default_value(1e9), "tuning frequency")
        ("txgain", po::value<double>(&txgain)->default_value(0), "TX gain")
        ("rxgain", po::value<double>(&rxgain)->default_value(0), "RX gain")
        ("ch_tx", po::value<int>(&ch_tx)->default_value(0), "TX channel select (-1 (none), 0 or 1)")
        ("ch_rx", po::value<int>(&ch_rx)->default_value(0), "RX channel select (-1 (none), 0 or 1)")
        ("wavefile", po::value<std::string>(&current_wavefile)->default_value("waveform_data.bin"), "path to waveform file")
        ("file", po::value<std::string>(&fname)->default_value("usrp_samples.dat"), "output data file")
        ("secs", po::value<double>(&seconds_in_future)->default_value(.1), "number of seconds in the future to receive")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(4096), "total number of samples to receive")
        ("rate", po::value<double>(&rate)->default_value(125e6), "rate of incoming samples")
        ("syncpps",po::value<bool>(&syncpps)->default_value(false), "specify to sync pulse time to pps edge")
        ("dilv", "specify to disable inner-loop verbose")
        ("npulses", po::value<size_t>(&npulses)->default_value(1), "total number of pulses to receive")
    ;
    // clang-format on
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help")) {
        std::cout << boost::format("UHD RX Timed Samples %s") % desc << std::endl;
        return ~0;
    }

    ch_select_t ch_select = {0x0};
    if (vm.count("ch_rx")){
      if (ch_rx == 0){
        ch_select.rx0 = 1;
      }
      else if (ch_rx == 1){
        ch_select.rx1 = 1;
      }
    }
    if (vm.count("ch_tx")){
      if (ch_tx == 0){
        ch_select.tx0 = 1;
      }
      else if (ch_tx == 1){
        ch_select.tx1 = 1;
      }
    }

    int err = usrpInit(args,timesrc,rate,freq,rxgain,txgain);
    if (err == EXIT_SUCCESS)
        std::cout<<"usrpInit completed successfully"<<std::endl;
    else{
        std::cerr<<"usrpInit returned error...Exiting"<<std::endl;
        return 1;
    }
    boost::filesystem::path p(fname.c_str());
    std::string base = p.stem().string();

    for (int i = 0; i < npulses; i++){
      double time_set = -1.0;
      if (syncpps){
        err = sync_pps(time_set,-1.0);
        if (err != 0) std::cerr << "Error: sync_pps returned: " << err << ". time_set: "<<time_set<<std::endl;
        // time_set-=.6;
      }
      std::vector<std::complex<short>> pulseVector;
      std::vector<uhd::rx_metadata_t> md_vec;
      unsigned long num_rx = total_num_samps;
      try{
            pulseStream(pulseVector,md_vec,num_rx,seconds_in_future,time_set,current_wavefile,ch_select);
      }
      catch(std::runtime_error &e){
          std::cerr<<std::endl<<"Error: PulseStream threw "<<e.what()<<std::endl;
          return 1;
      }
      std::string newfname;
      if (npulses>1){
        std::string basen = base + "-" + boost::lexical_cast<std::string>(i);
        boost::filesystem::path newpath = p.parent_path() / boost::filesystem::path(basen + p.extension().string());
        newfname = newpath.string();
      }
      else{
        newfname = fname;
      }
      std::ofstream file;
      file.open(newfname.c_str(), std::ofstream::binary);
      file.write((const char*)&pulseVector.front(), pulseVector.size()*sizeof(std::complex<short>));
      file.close();
    }
    // finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;

    return EXIT_SUCCESS;
}
