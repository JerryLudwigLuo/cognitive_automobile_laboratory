#ifndef CORRECT_TIME_OFFSET_HPP
#define CORRECT_TIME_OFFSET_HPP

inline void correct_time_offset(int64_t& timestamp)
{
  int64_t timestamp_offset = (KogniMobil::kogmo_timestamp_now() - timestamp);
  timestamp_offset = (int64_t)(round(timestamp_offset/3600.0e9))*3600e9;
  // double timestamp_offset_s =(double)(timestamp_offset/1e9);
  // int64_t h_off = (int64_t)round(timestamp_offset_s / (double)3600) ;
  // timestamp_offset = h_off * (int64_t)3600 * (int64_t)1e9;
  
  timestamp += timestamp_offset;
}


inline void correct_time_offset_from(int64_t& timestamp, int64_t ref_timestamp)
{
  int64_t timestamp_offset = (ref_timestamp - timestamp);
  timestamp_offset = (int64_t)(round(timestamp_offset/3600.0e9))*3600e9;
  timestamp += timestamp_offset;
}


#endif // CORRECT_TIME_OFFSET_HPP
