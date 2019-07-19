#pragma once

#include <string>
#include <iostream>
#include <boost/filesystem.hpp>
#include <generic_logger/generic_logger.hpp>


inline boost::filesystem::path normalizePath(const boost::filesystem::path &path) {
    boost::filesystem::path absPath = boost::filesystem::absolute(path);
    boost::filesystem::path::iterator it = absPath.begin();
    boost::filesystem::path result = *it++;

    /// Get canonical version of the existing part
    for (; boost::filesystem::exists(result / *it) && it != absPath.end(); ++it) {
        result /= *it;
    }
    result = boost::filesystem::canonical(result);
    /// For the rest remove ".." and "." in a path with no symlinks
    for (; it != absPath.end(); ++it) {
        /// Just move back on ../
        if (*it == "..") {
            result = result.parent_path();
        }
        /// Ignore "."
        else if (*it != ".") {
            /// Just cat other path entries
            result /= *it;
        }
    }
    return result;
}

inline std::string normalizePath(const std::string &path) {
    return  normalizePath( boost::filesystem::path(path) ).string();
}


inline bool existsDirectory(const boost::filesystem::path &path) {
    if(!boost::filesystem::is_directory( path ) ) {
        WARN_STREAM("Folder: " << path.string() << " does not exist");
        return false;
    }
    return true;
}

inline bool existsDirectory(const std::string &path) {
    return existsDirectory( boost::filesystem::path(path)  );
}

inline bool existsFile(const boost::filesystem::path &path ) {
    if(!boost::filesystem::exists( path )) {
        WARN_STREAM("File: " << path << " does not exist");
        return false;
    }
    return true;
}

inline bool existsFile(const std::string &path ) {
    return existsFile( boost::filesystem::path(path) );
}

inline void checkIsDirectory(const std::string &path) {
    if (existsDirectory(path)) return;
    throw std::runtime_error("Folder does not exist");
}

inline void checkIsFile(const std::string &path ) {
    if (existsFile(path)) return;
    throw std::runtime_error("File does not exist");
}


inline std::vector<boost::filesystem::path> getSubEntriesSorted(const std::string& folder) {
    std::vector<boost::filesystem::path> subEntries;
    if(!existsDirectory(folder) ) return subEntries;
    std::copy(boost::filesystem::directory_iterator(folder), boost::filesystem::directory_iterator(),
         back_inserter(subEntries));
    std::sort(subEntries.begin(), subEntries.end());
    return subEntries;
}

