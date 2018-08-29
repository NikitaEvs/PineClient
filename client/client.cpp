#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>
#include <iostream>

#include <complex>
#include <vector>
#include <algorithm>
#include <math.h>

#include <string>
#include <sstream>
#include <cstdio>
#include <stdio.h>

#include <wiringSerial.h>
#include "json/json.hpp"


using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;
typedef websocketpp::config::asio_client::message_type::ptr message_ptr;
typedef websocketpp::client<websocketpp::config::asio_client> client;

client* clientCurrent;
websocketpp::connection_hdl handlerCurrent;

using json = nlohmann::json;

const std::string url = "";
const int token = 1;
bool onSending = true;

std::string uartIn = "";

const double TwoPi = 6.283185307179586;
double AVal[2048]; 
double FTvl[1024];
int Nvl=2048;
int Nft=1024;

void FFTAnalysis(double *AVal, double *FTvl, int Nvl, int Nft) {
  int i, j, n, m, Mmax, Istp;
  double Tmpr, Tmpi, Wtmp, Theta;
  double Wpr, Wpi, Wr, Wi;
  double *Tmvl;

  n = Nvl * 2; Tmvl = new double[n];

  for (i = 0; i < n; i+=2) {
   Tmvl[i] = 0;
   Tmvl[i+1] = AVal[i/2];
  }

  i = 1; j = 1;
  while (i < n) {
    if (j > i) {
      Tmpr = Tmvl[i]; Tmvl[i] = Tmvl[j]; Tmvl[j] = Tmpr;
      Tmpr = Tmvl[i+1]; Tmvl[i+1] = Tmvl[j+1]; Tmvl[j+1] = Tmpr;
    }
    i = i + 2; m = Nvl;
    while ((m >= 2) && (j > m)) {
      j = j - m; m = m >> 1;
    }
    j = j + m;
  }

  Mmax = 2;
  while (n > Mmax) {
    Theta = -TwoPi / Mmax; Wpi = sin(Theta);
    Wtmp = sin(Theta / 2); Wpr = Wtmp * Wtmp * 2;
    Istp = Mmax * 2; Wr = 1; Wi = 0; m = 1;

    while (m < Mmax) {
      i = m; m = m + 2; Tmpr = Wr; Tmpi = Wi;
      Wr = Wr - Tmpr * Wpr - Tmpi * Wpi;
      Wi = Wi + Tmpr * Wpi - Tmpi * Wpr;

      while (i < n) {
        j = i + Mmax;
        Tmpr = Wr * Tmvl[j] - Wi * Tmvl[j-1];
        Tmpi = Wi * Tmvl[j] + Wr * Tmvl[j-1];

        Tmvl[j] = Tmvl[i] - Tmpr; Tmvl[j-1] = Tmvl[i-1] - Tmpi;
        Tmvl[i] = Tmvl[i] + Tmpr; Tmvl[i-1] = Tmvl[i-1] + Tmpi;
        i = i + Istp;
      }
    }

    Mmax = Istp;
  }

  for (i = 0; i < Nft; i++) {
    j = i * 2;
	FTvl[i] = 2*sqrt(pow(Tmvl[j],2) + pow(Tmvl[j+1],2))/Nvl;
  }

  delete []Tmvl;
}

void sendMsg(std::string msg) {
	websocketpp::lib::error_code errCode;
	clientCurrent -> send(handlerCurrent, msg, websocketpp::frame::opcode::text, errCode);
	if(errCode){
		std::cout << "Send message failed" << errCode.message() << std::endl;
	}
}

std::string serializeData(std::string data, int token) {
	json j;
	j["event"] = "data_pi";
	j["data"]["token"] = token;
	j["data"]["values"] = data;
	return j.dump();
}

void sendPkg() {
	std::string out="";
	std::stringstream ss;
	std::string num = "11";
	int inChr, count = 0;
	for(int i = 1 ; i <= 12;++i) {
		ss << i << ',';
	}
	ss << "Ans" << endl;
	if((inChr = serialOpen("/dev/ttyACM0", 9600)) < 0){
	}
	for(int q = 0; q < 100; ++q) {
		count = 0;
		double s = 0;
		while(count < 2049) {
		        char in = serialGetchar(inChr);
    			if(in == '\n') {
				if(count != 0) {
					double x = std::stod(uartIn);
					AVal[count - 1] = x;
					s = s + x * x;
				}
               			uartIn = "";
				count++;
        		} else {
                		uartIn += in;
        		}
		}	
		FFTAnalysis(AVal, FTvl, Nvl, Nft);
		double m;
		vector<double> d;
		for(int i = 37 ; i < 1004; i = i + 94) {
			m=0;
			for(int j = 0 ; j < 20; ++j) {
				m = max(m,FTvl[i+j]);
			}
			d.push_back(m);
		}
		d.push_back(sqrt(s/2048));
		for(int i = 0 ; i < d.size();++i) {
			ss << d[i] << ',';
		}
		ss << num << endl;
		cout << q + 1 << "/100" << endl;
	}
	out = ss.str();
	sendMsg(serializeData(out, token));
}

void sendingData() {
	while(onSending){
		sendPkg();
	}
}

void on_message(client* client, websocketpp::connection_hdl handler, message_ptr msg) {
	std::cout << "Handle msg" << std::endl;
	websocketpp::lib::error_code errCode;
	clientCurrent = client;
	handlerCurrent = handler;
	std::cout << "---------Receive msg---------" << std::endl;
	std::cout << msg -> get_payload() << std::endl;
	json j = json::parse(msg -> get_payload());
	std::string event = j["event"];
	std::cout << "Event: " << event << std::endl;
	if(event == "auth_req_s_m") {
		sendingData();
	} else if(event == "stop_data_pi") {
		onSending = false;
	}
	if(errCode){
		std::cout << "Handle msg failed" << std::endl;
	}
}

int main() {
	client client;
	try{
		client.set_access_channels(websocketpp::log::alevel::all);
		client.clear_access_channels(websocketpp::log::alevel::frame_payload);
		client.init_asio();
		websocketpp::lib::error_code errCode;
		client.set_message_handler(bind(&on_message,&client,::_1,::_2));
		client::connection_ptr connectionPtr = client.get_connection(uri, errCode);
		std::cout << "---------Connected---------" << std::endl;
		if(errCode){
			std::cout << "Error with connections: " << errCode.message() << std::endl;
			return 0;
		}
		std::cout << "No errCode" << std::endl;
		client.connect(connectionPtr);
		std::cout << "Client connected" << std::endl;
		client.run();
		std::cout << "Running" << std::endl;
	} catch (websocketpp::exception const & e) {
		std::cout << "Err code: " << e.what() << std::endl;
	}
	return 0;
}
