#ifndef AMPL_AMPLEXCEPTION_H
#define AMPL_AMPLEXCEPTION_H

#include <cstdlib>
#include <stdexcept>
#include <string>

#include "ampl/format.h"

namespace ampl {
/**
 * Represent an exception generated by %AMPL
 */
class AMPLException : public std::runtime_error {
 public:
  /**
  Copy constructor
  */
  AMPLException(const AMPLException &e) : std::runtime_error(e.what()) {
    message_ = e.message_;
    line_ = e.line_;
    offset_ = e.offset_;
    filename_ = e.filename_;
  }
  /**
  Constructor
  \param cause Cause of the exception
  */
  explicit AMPLException(fmt::CStringRef cause = fmt::CStringRef(""))
      : std::runtime_error(cause.c_str()) {
    line_ = -1;
    offset_ = -1;
    parseMessage(cause);
  }

  /**
  Constructor for %AMPL errors
  \param filename Name of the file in which the exception occurred
  \param row Row where the error is detected
  \param offset Offset where the error is detected
  \param message Message to be embedded in the exception
  */
  AMPLException(fmt::CStringRef filename, int row, int offset,
                fmt::CStringRef message)
      : std::runtime_error(
            getWhat(filename.c_str(), row, offset, message.c_str())),
        filename_(filename.c_str()),
        message_(message.c_str()) {
    line_ = row;
    offset_ = offset;
  }

  /**
  Here to avoid the error looser throw specifier for virtual
  AMPLException::~AMPLException()
  in some old c++ compilers
  */
  ~AMPLException() throw() {}

  /**
  Get the name of the file where the error was detected
  */
  const std::string &getSourceName() const { return filename_; }

  /**
  Get the row where the error is located
  */
  int getLineNumber() const { return line_; }

  /**
  Get the offset where the error is located
  */
  int getOffset() const { return offset_; }

  /**
  Get the error message
  */
  const std::string &getMessage() const { return message_; }
  /**
  Overwrites the source name for this error
  */
  void setSourceName(const std::string &sourceName) { filename_ = sourceName; }

 private:
  std::string filename_;
  int line_;
  int offset_;
  std::string message_;

  static std::string getWhat(fmt::StringRef filename, int row, int offset,
                             fmt::StringRef message) {
    fmt::MemoryWriter out;
    if (filename.size() > 0) out << "file " << filename << "\n";
    if (row >= 0) {
      out << "line " << row << " offset " << offset << "\n";
    }
    out << message << "\n";
    return out.str();
  }
  template <std::size_t SIZE>
  const char *skip(const char *&cursor, const char (&str)[SIZE]) {
    const char *s = std::strstr(cursor, str);
    if (s) cursor = s + SIZE - 1;
    return s;
  }
  template <std::size_t SIZE>
  int getIntegerAfterMarker(const char *&cursor, const char (&marker)[SIZE]) {
    int value = 0;
    char *end = 0;
    if (skip(cursor, marker)) {
      value = std::strtol(cursor, &end, 10);
      cursor = end;
    }
    return value;
  }
  void parseMessage(fmt::CStringRef msg) {
    try {
      const char *cursor = msg.c_str();
      if (skip(cursor, "file ")) {
        if (const char *end = std::strchr(cursor, '\n')) {
          filename_.assign(cursor, end);
          cursor = end + 1;
        }
        line_ = getIntegerAfterMarker(cursor, "line ");
        offset_ = getIntegerAfterMarker(cursor, "offset ");
        skip(cursor, "\n");
      }
      message_.assign(cursor, std::strlen(cursor) - 1);
    } catch (...) {
      message_ = msg.c_str();
    }
  }
};
/**
 * Thrown if AMPL could not be started because of licensing errors
 */
class LicenseException : public std::runtime_error {
 public:
  /**
  Constructor
  */
  LicenseException(const std::string &cause) : std::runtime_error(cause) {}
};

/**
 * Thrown if a file could not be found
 */
class FileIOException : public std::runtime_error {
 public:
  /**
  Constructor
  */
  FileIOException(const std::string &cause) : std::runtime_error(cause) {}
};

/**
Thrown when accessing entities with the wrong number of indices,
e.g. accessing an indexed entity with Entity.get()
*/
class UnsupportedOperationException : public std::runtime_error {
 public:
  /**
  Constructor
  */
  UnsupportedOperationException(const std::string &cause)
      : std::runtime_error(cause) {}
};
/**
Thrown when an invalid subscript is accessed
*/
class InvalidSubscriptException : public AMPLException {
 public:
  /**
  Constructor
  */
  InvalidSubscriptException(fmt::CStringRef filename, int row, int offset,
                            fmt::CStringRef message)
      : AMPLException(filename, row, offset, message) {}
};
/**
Thrown when a syntax error is detected
*/
class SyntaxErrorException : public AMPLException {
 public:
  /**
  Constructor
  */
  SyntaxErrorException(fmt::CStringRef filename, int row, int offset,
                       fmt::CStringRef message)
      : AMPLException(filename, row, offset, message) {}
};
/**
Thrown when an entity is accessed when no data has been
assigned yet
*/
class NoDataException : public std::runtime_error {
 public:
  /**
  Constructor
  */
  NoDataException(fmt::CStringRef msg) : std::runtime_error(msg.c_str()) {}
};

}  // namespace ampl

#endif