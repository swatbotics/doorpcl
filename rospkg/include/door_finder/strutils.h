/********************************************************/
/*    Author: Matt Zucker <mzucker1@swarthmore.edu>     */
/* Copyright (c) 2013 Matt Zucker. All rights reserved. */
/********************************************************/

#ifndef _STRUTILS_H_
#define _STRUTILS_H_

#include <string>
#include <ctype.h>

inline std::string directoryOf(const std::string& filename) {

  size_t pos = filename.rfind('/');

  if (pos == size_t(-1)) { 
    return ".";
  } else {
    return filename.substr(0, pos);
  }

}

inline std::string combineDir(const std::string& dir, const std::string& rel) {
  if (rel.length() && rel[0] == '/') {
    return rel;
  } else {
    return dir + "/" + rel;
  }
}

inline std::string lower(const std::string& s) {
  std::string rval = s;
  for (size_t i=0; i<s.length(); ++i) { rval[i] = tolower(rval[i]); }
  return rval;
}

inline std::string upper(const std::string& s) {
  std::string rval = s;
  for (size_t i=0; i<s.length(); ++i) { rval[i] = toupper(rval[i]); }
  return rval;
}


inline std::string trimws(const std::string& s) {
 
  size_t p0 = 0; 
  while (p0 < s.length() && isspace(s[p0])) { ++p0; } 

  size_t p1 = s.length()-1; 
  while (p1 > p0 && isspace(s[p1])) { --p1; } 

  return s.substr(p0, p1-p0+1);

}

inline bool split(const std::string& s, char c, 
                  std::string& s1, std::string& s2) {
  
  size_t p = s.find(c);
  if (p == std::string::npos) { return false; }

  s1 = trimws(s.substr(0, p));
  s2 = trimws(s.substr(p+1, s.length()-p-1));

  return true;

}

#endif
