// SHEEEEEEEE
#include "main.h"
#include <bits/stdc++.h>
#include <vector>
#include "globals.hpp"

std::map<std::string, int> config;
std::vector<std::string> mapKeys;


void parseConfig(std::string input) {
	std::vector <std::string> tokens;
	std::stringstream check1(input);
	std::string inter;

	try
	{
		// tokens is now each line
		while (std::getline(check1, inter, '\n')) {

			tokens.push_back(inter);
		}


		


    // iter through lines
	for (int i = 0; i < tokens.size(); i++) {
		try
		{
			std::string line = tokens[i];
			std::string id;
			std::string data;


				
			// tokenize data
			std::stringstream lineCheck(line);
			std::getline(lineCheck, id, ':');
			std::getline(lineCheck, data, ':');


			if (line == "\n") {
				continue;
			}

			// set to config
			mapKeys.push_back(id);
			config.insert(std::pair<std::string, int>(id, std::stoi(data)));
		}
		catch (...)
		{
			std::cout << "didnt like for";
		}

	}
	}
	catch (...)
	{
		std::cout << "didnt like";
	}

}

void initConfig() {
	try {
		// Read and parse
		FILE* usd_file_read = fopen("/usd/config.gcvc", "r");
		char buffer[50];
		fread(buffer, 1, 50, usd_file_read);
		parseConfig(buffer);
		fclose(usd_file_read); 
		std::cout << "Initialized Config...";
	} catch (...) {
		std::cout << "INIT ERROR" << std::endl;
	}


}

int getConfig(std::string s) {
	return config[s];
}

void setConfig(std::string s, int i) {
	try {
		// config.insert(std::pair<std::string, int>(s, i));
		config[s] = i;

		std::string sendToFile = "";

		for (std::string mapKey : mapKeys) {
			sendToFile += mapKey + ": " + std::to_string((int) getConfig(mapKey)) + "\n";
		}
		FILE* usd_file_write = fopen("/usd/config.gcvc", "w");
		sendToFile = sendToFile.substr(0, sendToFile.size() - 1);
		fputs(sendToFile.c_str(), usd_file_write);
		fclose(usd_file_write);
	} catch(...) {
		std::cout << "SET ERROR" << std::endl;
	}



}

