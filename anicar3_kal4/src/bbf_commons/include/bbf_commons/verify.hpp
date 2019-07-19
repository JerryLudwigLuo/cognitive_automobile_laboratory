/***************************************************************************
 *                                                                         *
 *   Author: Julius Ziegler                                                *
 *                                                                         *
 *   Copyright (C) 2014 by Atlatec UG (haftungsbeschraenkt)                *
 *                                                                         *
 *   http://atlatec.de                                                     *
 *                                                                         *
 ***************************************************************************/


#ifndef VERIFY_HPP
#define VERIFY_HPP

#include <string>
#include <stdexcept>

#define verify( expr, message ) { if( !(expr) ) throw std::runtime_error( std::string( "verify failed: " ) + #expr + "; " + message ); }

#endif // VERIFY_HPP
