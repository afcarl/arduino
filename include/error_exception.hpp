/*
 * Author: Benedict R. Gaster
 * Date: June 2013
 * Desc: Simple exception class used for errors
 *
 * Copyright 2014 Benedict R. Gaster
 * License: See the file license.
 */

#pragma once

#include <exception>
#include <string>

class error_exception
{
private:
	std::string err_str_;
public:
  error_exception(std::string err_str) : 
    err_str_(err_str)
  {}

  ~error_exception() throw() {}

  virtual std::string what() const throw()
  {
    if (err_str_.length() == 0) {
      return "empty";
    }
    else {
      return err_str_;
    }
  }
};
