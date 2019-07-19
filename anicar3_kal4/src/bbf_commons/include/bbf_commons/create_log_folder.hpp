#ifndef CREATE_LOG_FOLDER_HPP
#define CREATE_LOG_FOLDER_HPP

#include <boost/filesystem.hpp>
#include <boost/date_time.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <string>

std::string create_log_folder( std::string base_dir )
{
  auto _path = boost::filesystem::path( base_dir );
  boost::posix_time::ptime now(
        boost::posix_time::second_clock::local_time());

  auto symlink_path = _path;
  symlink_path /= "last";

  _path /= boost::posix_time::to_iso_string(now);

  boost::filesystem::remove(symlink_path);
  boost::filesystem::create_directories(_path);
  boost::filesystem::create_symlink(_path, symlink_path);

  return symlink_path.string();
}

#endif // CREATE_LOG_FOLDER_HPP
