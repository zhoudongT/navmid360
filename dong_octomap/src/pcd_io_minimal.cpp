#include "pcd_io_minimal.hpp"

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>

#include <unordered_map>

#include <zlib.h>

namespace dong_octomap_pcd
{
namespace
{

void trim(std::string & s)
{
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) {
    s.erase(s.begin());
  }
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) {
    s.pop_back();
  }
}

std::vector<std::string> split_ws(const std::string & line)
{
  std::vector<std::string> out;
  std::istringstream iss(line);
  std::string tok;
  while (iss >> tok) {
    out.push_back(tok);
  }
  return out;
}

struct Header
{
  std::vector<std::string> fields;
  std::vector<int> sizes;
  std::vector<char> types;
  std::vector<int> counts;
  int width{0};
  int height{1};
  size_t points{0};
  enum class DataKind { ASCII, BINARY, BINARY_COMPRESSED } data{DataKind::ASCII};
};

bool type_field_bytes(char type, int declared_size, int * out_bytes)
{
  switch (type) {
    case 'F':
      *out_bytes = 4;
      return declared_size == 4;
    case 'D':
      *out_bytes = 8;
      return declared_size == 8;
    case 'I':
    case 'U':
      *out_bytes = declared_size;
      return declared_size == 1 || declared_size == 2 || declared_size == 4 || declared_size == 8;
    default:
      return false;
  }
}

size_t field_index(const Header & h, const char * name)
{
  for (size_t i = 0; i < h.fields.size(); ++i) {
    if (h.fields[i] == name) {
      return i;
    }
  }
  return h.fields.size();
}

bool read_field_as_double(const char * p, char type, int size, double * out)
{
  if (type == 'F' && size == 4) {
    float v;
    std::memcpy(&v, p, 4);
    *out = static_cast<double>(v);
    return true;
  }
  if (type == 'D' && size == 8) {
    double v;
    std::memcpy(&v, p, 8);
    *out = v;
    return true;
  }
  if ((type == 'I' || type == 'U') && (size == 1 || size == 2 || size == 4 || size == 8)) {
    int64_t v = 0;
    if (size == 1) {
      int8_t s;
      std::memcpy(&s, p, 1);
      v = s;
    } else if (size == 2) {
      int16_t s;
      std::memcpy(&s, p, 2);
      v = s;
    } else if (size == 4) {
      int32_t s;
      std::memcpy(&s, p, 4);
      v = s;
    } else {
      std::memcpy(&v, p, 8);
    }
    *out = static_cast<double>(v);
    return true;
  }
  return false;
}

bool parse_header(std::istream & in, Header & h, std::string & err)
{
  std::string line;
  while (std::getline(in, line)) {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    trim(line);
    if (line.empty() || line[0] == '#') {
      continue;
    }
    const auto tok = split_ws(line);
    if (tok.empty()) {
      continue;
    }
    const std::string & key = tok[0];
    if (key == "VERSION" || key == "VIEWPOINT") {
      continue;
    }
    if (key == "FIELDS") {
      h.fields.assign(tok.begin() + 1, tok.end());
      continue;
    }
    if (key == "SIZE") {
      h.sizes.clear();
      for (size_t i = 1; i < tok.size(); ++i) {
        h.sizes.push_back(std::stoi(tok[i]));
      }
      continue;
    }
    if (key == "TYPE") {
      h.types.clear();
      for (size_t i = 1; i < tok.size(); ++i) {
        if (tok[i].size() != 1) {
          err = "PCD: invalid TYPE";
          return false;
        }
        h.types.push_back(tok[i][0]);
      }
      continue;
    }
    if (key == "COUNT") {
      h.counts.clear();
      for (size_t i = 1; i < tok.size(); ++i) {
        h.counts.push_back(std::stoi(tok[i]));
      }
      continue;
    }
    if (key == "WIDTH") {
      h.width = std::stoi(tok[1]);
      continue;
    }
    if (key == "HEIGHT") {
      h.height = std::stoi(tok[1]);
      continue;
    }
    if (key == "POINTS") {
      h.points = static_cast<size_t>(std::stoul(tok[1]));
      continue;
    }
    if (key == "DATA") {
      if (tok.size() < 2) {
        err = "PCD: DATA line incomplete";
        return false;
      }
      if (tok[1] == "ascii") {
        h.data = Header::DataKind::ASCII;
      } else if (tok[1] == "binary") {
        h.data = Header::DataKind::BINARY;
      } else if (tok[1] == "binary_compressed") {
        h.data = Header::DataKind::BINARY_COMPRESSED;
      } else {
        err = "PCD: unsupported DATA: " + tok[1];
        return false;
      }
      break;
    }
  }

  if (h.fields.empty() || h.sizes.size() != h.fields.size() || h.types.size() != h.fields.size() ||
    h.counts.size() != h.fields.size())
  {
    err = "PCD: bad header (FIELDS/SIZE/TYPE/COUNT)";
    return false;
  }
  for (size_t i = 0; i < h.fields.size(); ++i) {
    int bs = 0;
    if (!type_field_bytes(h.types[i], h.sizes[i], &bs)) {
      err = "PCD: unsupported field type";
      return false;
    }
  }
  if (h.points == 0) {
    if (h.width <= 0 || h.height <= 0) {
      err = "PCD: missing POINTS/WIDTH/HEIGHT";
      return false;
    }
    h.points = static_cast<size_t>(h.width) * static_cast<size_t>(h.height);
  }
  return true;
}

size_t point_row_bytes(const Header & h)
{
  size_t row = 0;
  for (size_t i = 0; i < h.fields.size(); ++i) {
    int bs = 0;
    type_field_bytes(h.types[i], h.sizes[i], &bs);
    row += static_cast<size_t>(bs) * static_cast<size_t>(h.counts[i]);
  }
  return row;
}

bool offsets_xyz(const Header & h, size_t * ox, size_t * oy, size_t * oz, std::string & err)
{
  size_t off = 0;
  bool hx = false, hy = false, hz = false;
  for (size_t i = 0; i < h.fields.size(); ++i) {
    int bs = 0;
    type_field_bytes(h.types[i], h.sizes[i], &bs);
    const size_t span = static_cast<size_t>(bs) * static_cast<size_t>(h.counts[i]);
    if (h.fields[i] == "x") {
      *ox = off;
      hx = true;
    } else if (h.fields[i] == "y") {
      *oy = off;
      hy = true;
    } else if (h.fields[i] == "z") {
      *oz = off;
      hz = true;
    }
    off += span;
  }
  if (!hx || !hy || !hz) {
    err = "PCD: need x y z fields";
    return false;
  }
  return true;
}

bool load_ascii(std::istream & in, const Header & h, std::vector<std::array<double, 3>> & out, std::string & err)
{
  const size_t ix = field_index(h, "x");
  const size_t iy = field_index(h, "y");
  const size_t iz = field_index(h, "z");
  if (ix >= h.fields.size() || iy >= h.fields.size() || iz >= h.fields.size()) {
    err = "PCD: missing x/y/z";
    return false;
  }
  size_t tok_x = 0;
  for (size_t fi = 0; fi < ix; ++fi) {
    tok_x += static_cast<size_t>(h.counts[fi]);
  }
  size_t tok_y = 0;
  for (size_t fi = 0; fi < iy; ++fi) {
    tok_y += static_cast<size_t>(h.counts[fi]);
  }
  size_t tok_z = 0;
  for (size_t fi = 0; fi < iz; ++fi) {
    tok_z += static_cast<size_t>(h.counts[fi]);
  }
  size_t n_tokens = 0;
  for (size_t fi = 0; fi < h.fields.size(); ++fi) {
    n_tokens += static_cast<size_t>(h.counts[fi]);
  }
  out.clear();
  out.reserve(h.points);
  std::string line;
  for (size_t pi = 0; pi < h.points; ++pi) {
    do {
      if (!std::getline(in, line)) {
        err = "PCD: EOF in ASCII data";
        return false;
      }
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      trim(line);
    } while (line.empty());
    const auto tok = split_ws(line);
    if (tok.size() < n_tokens) {
      err = "PCD: short ASCII line";
      return false;
    }
    out.push_back(
      {{std::stod(tok[tok_x]), std::stod(tok[tok_y]), std::stod(tok[tok_z])}});
  }
  return true;
}

bool load_binary_rows(
  const char * data, size_t nbytes, const Header & h, std::vector<std::array<double, 3>> & out, std::string & err)
{
  size_t ox = 0, oy = 0, oz = 0;
  if (!offsets_xyz(h, &ox, &oy, &oz, err)) {
    return false;
  }
  const size_t ix = field_index(h, "x");
  const size_t iy = field_index(h, "y");
  const size_t iz = field_index(h, "z");
  const size_t row = point_row_bytes(h);
  if (row == 0 || nbytes != row * h.points) {
    err = "PCD: binary size mismatch";
    return false;
  }
  out.clear();
  out.reserve(h.points);
  for (size_t pi = 0; pi < h.points; ++pi) {
    const char * rowp = data + pi * row;
    double vx = 0, vy = 0, vz = 0;
    if (!read_field_as_double(rowp + ox, h.types[ix], h.sizes[ix], &vx) ||
      !read_field_as_double(rowp + oy, h.types[iy], h.sizes[iy], &vy) ||
      !read_field_as_double(rowp + oz, h.types[iz], h.sizes[iz], &vz))
    {
      err = "PCD: binary decode failed";
      return false;
    }
    out.push_back({{vx, vy, vz}});
  }
  return true;
}

}  // namespace

bool load_pcd_xyz(
  const std::string & path,
  std::vector<std::array<double, 3>> & out_points,
  std::string & err_msg)
{
  out_points.clear();
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    err_msg = "cannot open file";
    return false;
  }
  Header h;
  if (!parse_header(in, h, err_msg)) {
    return false;
  }

  if (h.data == Header::DataKind::ASCII) {
    return load_ascii(in, h, out_points, err_msg);
  }

  if (h.data == Header::DataKind::BINARY) {
    const size_t row = point_row_bytes(h);
    std::vector<char> buf(row * h.points);
    in.read(buf.data(), static_cast<std::streamsize>(buf.size()));
    if (!in || static_cast<size_t>(in.gcount()) != buf.size()) {
      err_msg = "PCD: binary read failed";
      return false;
    }
    return load_binary_rows(buf.data(), buf.size(), h, out_points, err_msg);
  }

  uint32_t cmp_size = 0;
  uint32_t dec_size = 0;
  in.read(reinterpret_cast<char *>(&cmp_size), 4);
  in.read(reinterpret_cast<char *>(&dec_size), 4);
  if (!in) {
    err_msg = "PCD: compressed header read failed";
    return false;
  }
  std::vector<unsigned char> compressed(cmp_size);
  in.read(reinterpret_cast<char *>(compressed.data()), cmp_size);
  if (!in || static_cast<size_t>(in.gcount()) != cmp_size) {
    err_msg = "PCD: compressed data read failed";
    return false;
  }
  std::vector<unsigned char> dec(dec_size);
  uLongf destlen = dec_size;
  const int zr = uncompress(dec.data(), &destlen, compressed.data(), cmp_size);
  if (zr != Z_OK || destlen != dec_size) {
    err_msg = "PCD: zlib uncompress failed";
    return false;
  }
  return load_binary_rows(
    reinterpret_cast<const char *>(dec.data()), dec_size, h, out_points, err_msg);
}

}  // namespace dong_octomap_pcd
