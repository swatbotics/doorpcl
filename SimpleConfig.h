/********************************************************/
/*    Author: Matt Zucker <mzucker1@swarthmore.edu>     */
/* Copyright (c) 2013 Matt Zucker. All rights reserved. */
/********************************************************/

#ifndef _SIMPLECONFIG_H_
#define _SIMPLECONFIG_H_

#include <string>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <stdio.h>
#include "strutils.h"

class SimpleConfig {
public:

  typedef std::map<std::string, std::pair<std::string, bool> > StringMap;

  StringMap lookup;

  SimpleConfig(const std::string& filename) { 
    std::ifstream istr(filename.c_str());
    if (!istr.is_open()) { 
      std::cerr << "error opening config " << filename << "\n";
      exit(1);
    }
    parse(istr);
  }

  SimpleConfig(std::istream& istr) {
    parse(istr);
  }

  void parse(std::istream& istr) {

    std::string line;

    while (std::getline(istr,line)) {
      size_t p = line.find('#');
      if (p != std::string::npos) { 
        line = line.substr(0, p);
      }
      std::string s1, s2;
      if (!split(line, '=', s1, s2)) {
        if (!trimws(line).empty()) {
          std::cerr << "error parsing line " << line << "\n";
          exit(1);
        }
      } else {
        std::cout << "set '" << s1 << "' to '" << s2 << "'\n";
        lookup[s1] = std::make_pair(s2, false);
      }
    }

  }

  const std::string& get(const std::string& key) {
    StringMap::iterator i = lookup.find(key);
    if (i == lookup.end()) { 
      std::cerr << "config key not found: " << key << "\n";
      exit(1);
    }
    i->second.second = true;
    return i->second.first;
  }

  template <class Tval>
  void get(const std::string& key, Tval& val) {

    const std::string& sval = get(key);

    std::istringstream istr(sval);

    if ( !(istr >> val) || (istr.peek() != EOF)) {
      std::cerr << "error parsing value for " << key << "\n";
      exit(1);
    }

  }

  bool getBool(const std::string& key) { 
    const std::string& sval = lower(get(key));
    if (sval == "true" || sval == "1") { 
      return true;
    } else if (sval == "false" || sval == "0") {
      return false;
    } else {
      std::cerr << "error parsing boolean value for " << key << "\n";
      exit(1);
    }
  }

  void checkUsed(bool abortIfUnused) const {
    bool unused = false;
    for (StringMap::const_iterator i = lookup.begin(); i!=lookup.end(); ++i) {
      if (!i->second.second) { 
        std::cerr << "*** CONFIG ITEM " << i->first << " WAS SET BUT NEVER READ! ***\n";
        unused = true;
      }
    }
    if (unused && abortIfUnused) {
      exit(1);
    }
  }

};

#endif
