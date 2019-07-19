#ifndef SPLIT_STRING_H
#define SPLIT_STRING_H

#include <string>
#include <vector>
#include <functional>
#include <iostream>
#include <math.h>

void split_double(const std::string& s, char c, std::vector<double>& v) {
   std::string::size_type i = 0;
   std::string::size_type j = s.find(c);

   if (j == std::string::npos)
     v.push_back(atof(s.c_str()));

   while (j != std::string::npos) {
      v.push_back(atof(s.substr(i, j-i).c_str()));
      i = ++j;
      j = s.find(c, j);
      if (j == std::string::npos)
         v.push_back(atof(s.substr(i, s.length( )).c_str()));
   }
}

void split_string(const std::string& s, char c, std::vector<std::string>& v) {
   std::string::size_type i = 0;
   std::string::size_type j = s.find(c);

   if (j == std::string::npos)
     v.push_back(s);

   while (j != std::string::npos) {
      v.push_back(s.substr(i, j-i));
      i = ++j;
      j = s.find(c, j);
      if (j == std::string::npos)
         v.push_back(s.substr(i, s.length( )));
   }
}

#endif // SPLIT_STRING_H
