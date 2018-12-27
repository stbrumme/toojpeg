// //////////////////////////////////////////////////////////
// toojpeg.cpp
// written by Stephan Brumme, 2018
// see https://create.stephan-brumme.com/toojpeg/
//

#include "toojpeg.h"

// notes:
// - the popular STB Image library includes Jon's code as well:
//   https://github.com/nothings/stb/blob/master/stb_image_write.h
// - a short documentation of the JFIF/JPEG file format can be found here:
//   https://en.wikipedia.org/wiki/JPEG_File_Interchange_Format
// - the most readable JPEG book (from a developer's perspective) is Miano's "Compressed Image File Formats" (1999, ISBN 0-201-60443-4),
//   used copies are really cheap nowadays and it includes a CD with C++ sources as well (plus detailled format descriptions of GIF+PNG)
// - much more detailled is Mitchell/Pennebaker's "JPEG: Still Image Data Compression Standard" (1993, ISBN 0-442-01272-1)
//   which contains the official JPEG standard - fun fact: I accidentally bought a signed copy

namespace // anonymous namespace to hide local functions / constants / etc.
{
// ////////////////////////////////////////
// data types (for internal use only)
// one byte
typedef unsigned char   uint8_t;
// two bytes
typedef unsigned short uint16_t;
typedef          short  int16_t;
// four bytes (or more)
typedef unsigned int   uint32_t;
typedef          int    int32_t;

// Huffman code
struct HuffmanCode
{
  HuffmanCode(uint16_t code_ = 0, uint8_t numBits_ = 0)
  : code(code_), numBits(numBits_) {}
  uint16_t code;     // JPEG's Huffman codes are limited to 16 bits
  uint8_t  numBits;  // actual number of bits
};

// store the most recently encoded bits that are not written yet
struct BitBuffer
{
  BitBuffer()        // there will be only one instance of this object
  : bits(0), numBits(0) {}
  uint32_t bits;     // actually only at most 24 bits are used
  uint8_t  numBits;  // number of valid bits (the right-most bits)
};

// ////////////////////////////////////////
// constants

// 8x8 blocks are processed in zig-zag order
static const uint8_t ZigZag[] =
    {  0, 1, 5, 6,14,15,27,28,
       2, 4, 7,13,16,26,29,42,
       3, 8,12,17,25,30,41,43,
       9,11,18,24,31,40,44,53,
      10,19,23,32,39,45,52,54,
      20,22,33,38,46,51,55,60,
      21,34,37,47,50,56,59,61,
      35,36,48,49,57,58,62,63 };

// some constants for our DCT
const auto SqrtHalfSqrt = 1.306562965f; // sqrt((2 + sqrt(2)) / 2)  = cos(pi * 1 / 8) * sqrt(2)
const auto HalfSqrtSqrt = 0.382683433f; // sqrt( 2 - sqrt(2)) / 2   = cos(pi * 3 / 8)
const auto InvSqrt      = 0.707106781f; // 1 / sqrt(2)              = cos(pi * 2 / 8)
const auto InvSqrtSqrt  = 0.541196100f; // 1 / sqrt(2 - sqrt(2))    = cos(pi * 3 / 8) * sqrt(2)

// ////////////////////////////////////////
// helper functions (for internal use only)

// restrict a value to the interval [minimum, maximum]
template <typename T>
T clamp(T value, int minimum, int maximum)
{
  if (value <= minimum) return minimum;
  if (value >= maximum) return maximum;
  return value;
}

// start a new JFIF block
void writeMarker(TooJpeg::WRITE_ONE_BYTE output, uint8_t id, uint16_t length)
{
  output(0xFF); output(id);      // ID, always preceded by 0xFF
  output(uint8_t(length >> 8));  // length (big-endian)
  output(uint8_t(length & 0xFF));
}

// write bits stored in BitCode, keep excess bits in BitBuffer
void writeBits(TooJpeg::WRITE_ONE_BYTE output, BitBuffer& buffer, HuffmanCode data)
{
  // append the new bits to those bits leftover from previous call(s)
  buffer.numBits += data.numBits;
  buffer.bits   <<= data.numBits;
  buffer.bits    |= data.code;

  // write all "full" bytes
  while (buffer.numBits >= 8)
  {
    // extract highest 8 bits
    buffer.numBits -= 8;
    uint8_t oneByte = (buffer.bits >> buffer.numBits) & 0xFF;
    output(oneByte);

    if (oneByte == 0xFF) // 0xFF has a special meaning for JPEGs (it's a block marker)
      output(0);         // therefore pad a zero to indicate "nope, this one ain't a marker, it's just a coincidence"

    // note: I don't clear those written bits, therefore buffer.bits contains garbage in the high bits
    //       if you really want to "clean up" (e.g. for debugging purposes) then uncomment the following line
    //buffer.bits &= (1 << buffer.numBits) - 1;
  }
}

// convert to Huffman code
HuffmanCode convertCode(int16_t value)
{
  // strip sign
  int16_t absolute = value < 0 ? -value  : +value;

  // find highest 1-bit
  uint8_t numBits = 0;
  while (absolute != 0)
  {
    absolute >>= 1;
    numBits++;
  }

  if (value < 0)
    value--;

  // remove any excess bits
  auto mask = int16_t((1 << numBits) - 1);
  auto code = value & mask;
  return HuffmanCode((uint16_t)code, numBits);
}

// forward DCT computation (fast AAN algorithm: Arai, Agui and Nakajima: "A fast DCT-SQ scheme for images")
void DCT(float* block, uint8_t stride) // stride = 1 or 8 (horizontal or vertical)
{
  // modify in-place
  auto& block0 = block[0 * stride];
  auto& block1 = block[1 * stride];
  auto& block2 = block[2 * stride];
  auto& block3 = block[3 * stride];
  auto& block4 = block[4 * stride];
  auto& block5 = block[5 * stride];
  auto& block6 = block[6 * stride];
  auto& block7 = block[7 * stride];

  // based on https://dev.w3.org/Amaya/libjpeg/jidctflt.c
  auto add07 = block0 + block7; auto sub07 = block0 - block7;
  auto add16 = block1 + block6; auto sub16 = block1 - block6;
  auto add25 = block2 + block5; auto sub25 = block2 - block5;
  auto add34 = block3 + block4; auto sub34 = block3 - block4;

  auto add0347 = add07 + add34; auto sub07_34 = add07 - add34;
  auto add1256 = add16 + add25; auto sub16_25 = add16 - add25;

  block0 = add0347 + add1256; block4 = add0347 - add1256;

  auto z1 = (sub16_25 + sub07_34) * InvSqrt;
  block2 = sub07_34 + z1; block6 = sub07_34 - z1;

  auto sub23_45 = sub34 + sub25;
  auto sub01_67 = sub16 + sub07;

  auto z2 = (sub23_45 - sub01_67) * HalfSqrtSqrt;
  auto z3 = (sub25    + sub16   ) * InvSqrt;
  auto z4 =  sub01_67 * SqrtHalfSqrt + z2;
  auto z5 =  sub23_45 * InvSqrtSqrt  + z2;
  auto z6 = sub07 + z3;
  auto z7 = sub07 - z3;
  block1 = z6 + z4; block7 = z6 - z4;
  block5 = z7 + z5; block3 = z7 - z5;
}

// process 8x8 block
int16_t processDU(TooJpeg::WRITE_ONE_BYTE output, BitBuffer& buffer,
                  float block[8][8], const float scaled[64], int16_t lastDC,
                  const HuffmanCode huffmanDC[256], const HuffmanCode huffmanAC[256])
{
  // "linearize" the 8x8 block, treat it as a flat array of 64 floats
  auto block64 = (float*) block;

  // DCT rows
  for (auto offset = 0; offset < 8; offset++)
    DCT(block64 + offset * 8, 1);

  // DCT columns
  for (auto offset = 0; offset < 8; offset++)
    DCT(block64 + offset * 1, 8);

  // quantize/scale/zigzag the coefficients
  int16_t Q[64];
  for (auto i = 0; i < 64; i++)
  {
    // scale
    auto q = block64[i] * scaled[i];
    // round to nearest integer (actually, rounding is performed in the next step by casting from float to int)
    q += (q > 0 ? +0.5f : -0.5f);

    // clamp to 16 bits
    Q[ZigZag[i]] = (int16_t)clamp(int32_t(q), -32768, +32767);
  }

  // encode DC (Q[0] is the "average color" of the 8x8 block)
  int16_t DC   = Q[0];
  int16_t diff = DC - lastDC;
  if (diff == 0)
    writeBits(output, buffer, huffmanDC[0]);
  else
  {
    auto bits = convertCode(diff);
    writeBits(output, buffer, huffmanDC[bits.numBits]);
    writeBits(output, buffer, bits);
  }

  // encode ACs (Q[1..63])
  auto posNonZero = 63;
  while (posNonZero > 0 && Q[posNonZero] == 0)
    posNonZero--;

  for (auto i = 1; i <= posNonZero; i++)
  {
    // skip zeros
    auto skipZeros = 0;
    while (Q[i] == 0 && i <= posNonZero)
    {
      i++;
      skipZeros++;
    }

    // encode upper 4 bits
    while (skipZeros >= 16)
    {
      writeBits(output, buffer, huffmanAC[0xF0]);
      skipZeros -= 16;
    }

    // convert lower 4 bits to Huffman code
    auto bits   = convertCode(Q[i]);
    auto offset = 16 * skipZeros + bits.numBits;
    writeBits(output, buffer, huffmanAC[offset]);
    writeBits(output, buffer, bits);
  }

  // send end-of-block code
  if (posNonZero < 63)
    writeBits(output, buffer, huffmanAC[0]);

  return DC;
}

// Jon's code includes the pre-generated Huffman codes
// I don't like these "magic constants" and compute them on my own :-)
void generateHuffmanTable(const uint8_t numCodes[16], const uint8_t* values, HuffmanCode result[256])
{
  // next Huffman code
  uint16_t code = 0;
  // process all bitsizes 1-16 ...
  for (uint8_t numBits = 1; numBits <= 16; numBits++)
  {
    // ... and each code of these bitsizes
    for (uint8_t i = 0; i < numCodes[numBits - 1]; i++) // note numCodes array starts at zero, but smallest bitsize is 1
    {
      auto current = *values++;
      result[current].code    = code++;
      result[current].numBits = numBits;
    }
    // next Huffman code needs to be one bit wider
    code <<= 1;
  }
}

} // end of anonymous namespace

// -------------------- the only externally visible function ... --------------------

namespace TooJpeg
{

// handle       - callback that stores a single byte (writes to disk, memory, ...)
// width,height - image size
// pixels       - stored in RGB format or grayscale, stored from upper-left to lower-right
// isRGB        - true if RGB format (3 bytes per pixel); false if grayscale (1 byte per pixel)
// quality      - between 1 (worst) and 100 (best)
// downSample   - if true then YCbCr 4:2:0 format is used (smaller size, minor quality loss) instead of 4:4:4, not relevant for grayscale
// comment      - optional JPEG comment (0/NULL if no comment)
bool writeJpeg(TooJpeg::WRITE_ONE_BYTE output, const void* pixels_, unsigned short width_, unsigned short height_,
               bool isRGB, unsigned char quality_, bool downSample, const char* comment)
{
  // reject invalid pointers
  if (!output || !pixels_)
    return false;
  // check image format
  if (width_ == 0 || height_ == 0)
    return false;

  // quality level
  unsigned int quality = quality_;
  if (quality == 0)
    quality = 1;
  if (quality >  100)
    quality = 100;

  quality = quality < 50 ? 5000 / quality : 200 - quality * 2;

  // number of components
  uint8_t numComponents = isRGB ? 3 : 1;
  // note: if there is just one component (=grayscale), then only luminance needs to be stored in the file
  //       thus everything related to chrominance need not to be written to the JPEG
  //       I still compute a few things, like quantization tables to avoid a complete code mess

  // grayscale images can't be downsampled (because there is no Cb + Cr ...)
  if (!isRGB)
    downSample = false;

  // ////////////////////////////////////////
  // JFIF headers
  static const uint8_t HeaderJfif[2+2+16] =
      { 0xFF,0xD8,         // SOI marker (start of image)
        0xFF,0xE0,         // JFIF APP0 tag
        0,16,              // length: 16 bytes (14 bytes payload + 2 bytes for this length field)
        'J','F','I','F',0, // JFIF identifier, zero-terminated
        1,1,               // JFIF version 1.1
        0,                 // no density units specified
        0,1,0,1,           // 1 pixel "per pixel" horizontally and vertically
        0,0 };             // no thumbnail (size 0 x 0)
  for (auto c : HeaderJfif)
    output(c);

  // ////////////////////////////////////////
  // comment (if requested)
  if (comment != 0)
  {
    // look for zero terminator
    auto scan = comment;
    while (*scan++) ; 

    // length: number of bytes (without zero terminator) + 2 bytes for this length field
    uint16_t length = 2 + scan - comment;
    writeMarker(output, 0xFE, length); // COM marker

    // ... and the comment itself
    while (length-- > 0)
      output(uint8_t(*comment++));
  }

  // ////////////////////////////////////////
  // write new quantization tables
  writeMarker(output, 0xDB, isRGB ? 2+2*65 : 2+1*65); // length: 65 bytes per table + 2 bytes for this length field
                                                      // each table has 64 entries and is preceded by an ID byte

  // quantization tables from JPEG Standard, Annex K
  // there are a few papers which propose slightly more efficient values
  // btw: Google's Guetzli project attempts to optimize these tables per image
  static const uint8_t DefaultQuantLuminance[64] =
                     { 16, 11, 10, 16, 24, 40, 51, 61,
                       12, 12, 14, 19, 26, 58, 60, 55,
                       14, 13, 16, 24, 40, 57, 69, 56,
                       14, 17, 22, 29, 51, 87, 80, 62,
                       18, 22, 37, 56, 68,109,103, 77,
                       24, 35, 55, 64, 81,104,113, 92,
                       49, 64, 78, 87,103,121,120,101,
                       72, 92, 95, 98,112,100,103, 99 };
  static const uint8_t DefaultQuantChrominance[64] =
                     { 17, 18, 24, 47, 99, 99, 99, 99,
                       18, 21, 26, 66, 99, 99, 99, 99,
                       24, 26, 56, 99, 99, 99, 99, 99,
                       47, 66, 99, 99, 99, 99, 99, 99,
                       99, 99, 99, 99, 99, 99, 99, 99,
                       99, 99, 99, 99, 99, 99, 99, 99,
                       99, 99, 99, 99, 99, 99, 99, 99,
                       99, 99, 99, 99, 99, 99, 99, 99 };

  // adjust quantization tables to desired quality
  uint8_t quantLuminance  [64];
  uint8_t quantChrominance[64];
  for (auto i = 0; i < 64; i++)
  {
    int luminance   = (DefaultQuantLuminance  [i] * quality + 50) / 100;
    int chrominance = (DefaultQuantChrominance[i] * quality + 50) / 100;

    // clamp to 1..255
    quantLuminance  [ZigZag[i]] = uint8_t(clamp(luminance,   1, 255));
    quantChrominance[ZigZag[i]] = uint8_t(clamp(chrominance, 1, 255));
  }

  // both tables need to be written to the JPEG ...
  output(0); // first  quantization table
  for (auto c : quantLuminance)
    output(c);
  if (isRGB) // chrominance is only relevant for color images
  {
    output(1); // second quantization table
    for (auto c : quantChrominance)
      output(c);
  }

  // ////////////////////////////////////////
  // write image infos (SOF0 - start of frame)
  writeMarker(output, 0xC0, 2+6+3*numComponents); // length: 6 bytes general info + 3 per channel + 2 bytes for this length field

  // 8 bits per channel
  output(8);

  // image dimensions (big-endian)
  output(uint8_t(height_ >> 8)); output(uint8_t(height_ &  0xFF));
  output(uint8_t(width_  >> 8)); output(uint8_t(width_  &  0xFF));

  // sampling and quantization tables for each component
  output(numComponents);          // 1 component (grayscale, Y only) or 3 components (Y,Cb,Cr)
  for (uint8_t i = 1; i <= numComponents; i++)
  {
    output(i);                    // component ID (Y=1, Cb=2, Cr=3)
    // bitmasks for sampling: highest 4 bits: horizontal, lowest 4 bits: vertical
    output(i == 1 && downSample ? 0x22 : 0x11); // 0x11 is default YCbCr 4:4:4 and 0x22 stands for YCbCr 4:2:0
    output(i == 1 ? 0 : 1);       // use quantization table 0 for Y, else table 1
  }

  // ////////////////////////////////////////
  // Huffman tables
  HuffmanCode huffmanLuminanceDC  [256];
  HuffmanCode huffmanLuminanceAC  [256];
  HuffmanCode huffmanChrominanceDC[256];
  HuffmanCode huffmanChrominanceAC[256];

  // DHT marker - define Huffman tables
  writeMarker(output, 0xC4, isRGB ? (2+2*208) : (2+208));
                                  //  2 bytes for the length field
                                  //   1+16+12  for the DC luminance
                                  //   1+16+162 for the AC luminance
                                  //   1+16+12  for the DC chrominance
                                  //   1+16+162 for the AC chrominance

  // Huffman definitions for first DC table
  static const uint8_t DcLuminanceCodesPerBitsize[16] = { 0,1,5,1,1,1,1,1,1,0,0,0,0,0,0,0 }; // sum = 12
  static const uint8_t DcLuminanceValues         [12] = { 0,1,2,3,4,5,6,7,8,9,10,11 };       // => 12 codes
  // Huffman definitions for first AC table
  static const uint8_t AcLuminanceCodesPerBitsize[16]  = { 0,2,1,3,3,2,4,3,5,5,4,4,0,0,1,125 }; // sum = 162
  static const uint8_t AcLuminanceValues        [162] =                                         // => 162 codes
      { 0x01,0x02,0x03,0x00,0x04,0x11,0x05,0x12,0x21,0x31,0x41,0x06,0x13,0x51,0x61,0x07,0x22,0x71,0x14,0x32,0x81,0x91,0xA1,0x08,
        0x23,0x42,0xB1,0xC1,0x15,0x52,0xD1,0xF0,0x24,0x33,0x62,0x72,0x82,0x09,0x0A,0x16,0x17,0x18,0x19,0x1A,0x25,0x26,0x27,0x28,
        0x29,0x2A,0x34,0x35,0x36,0x37,0x38,0x39,0x3A,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x53,0x54,0x55,0x56,0x57,0x58,0x59,
        0x5A,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6A,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x83,0x84,0x85,0x86,0x87,0x88,0x89,
        0x8A,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,0xA8,0xA9,0xAA,0xB2,0xB3,0xB4,0xB5,0xB6,
        0xB7,0xB8,0xB9,0xBA,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8,0xD9,0xDA,0xE1,0xE2,
        0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xF1,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF8,0xF9,0xFA };

  // store luminance's DC+AC Huffman table definitions
  output(0x00); // highest 4 bits: 0 => DC, lowest 4 bits: 0 => Y (baseline)
  for (auto c : DcLuminanceCodesPerBitsize)
    output(c);
  for (auto c : DcLuminanceValues)
    output(c);
  output(0x10); // highest 4 bits: 1 => AC, lowest 4 bits: 0 => Y (baseline)
  for (auto c : AcLuminanceCodesPerBitsize)
    output(c);
  for (auto c : AcLuminanceValues)
    output(c);

  // compute actual Huffman code tables (see Jon's code for precalculated tables)
  generateHuffmanTable(DcLuminanceCodesPerBitsize, DcLuminanceValues, huffmanLuminanceDC);
  generateHuffmanTable(AcLuminanceCodesPerBitsize, AcLuminanceValues, huffmanLuminanceAC);

  // chrominance is only relevant for color images
  if (isRGB)
  {
    // Huffman definitions for second DC table
    static const uint8_t DcChrominanceCodesPerBitsize[16] = { 0,3,1,1,1,1,1,1,1,1,1,0,0,0,0,0 }; // sum = 12
    const auto& DcChrominanceValues = DcLuminanceValues;//= { 0,1,2,3,4,5,6,7,8,9,10,11 };       // => 12 codes (identical to DcLuminanceValues)
    // Huffman definitions for second AC table
    static const uint8_t AcChrominanceCodesPerBitsize[16] = { 0,2,1,2,4,4,3,4,7,5,4,4,0,1,2,119 }; // sum = 162
    static const uint8_t AcChrominanceValues        [162] =                                        // => 162 codes
        { 0x00,0x01,0x02,0x03,0x11,0x04,0x05,0x21,0x31,0x06,0x12,0x41,0x51,0x07,0x61,0x71,0x13,0x22,0x32,0x81,0x08,0x14,0x42,0x91,
          0xA1,0xB1,0xC1,0x09,0x23,0x33,0x52,0xF0,0x15,0x62,0x72,0xD1,0x0A,0x16,0x24,0x34,0xE1,0x25,0xF1,0x17,0x18,0x19,0x1A,0x26,
          0x27,0x28,0x29,0x2A,0x35,0x36,0x37,0x38,0x39,0x3A,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x53,0x54,0x55,0x56,0x57,0x58,
          0x59,0x5A,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6A,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x82,0x83,0x84,0x85,0x86,0x87,
          0x88,0x89,0x8A,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,0xA8,0xA9,0xAA,0xB2,0xB3,0xB4,
          0xB5,0xB6,0xB7,0xB8,0xB9,0xBA,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8,0xD9,0xDA,
          0xE2,0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF8,0xF9,0xFA };

    // store luminance's DC+AC Huffman table definitions
    output(0x01); // highest 4 bits: 0 => DC, lowest 4 bits: 1 => Cr,Cb (baseline)
    for (auto c : DcChrominanceCodesPerBitsize)
      output(c);
    for (auto c : DcChrominanceValues)
      output(c);
    output(0x11); // highest 4 bits: 1 => AC, lowest 4 bits: 1 => Cr,Cb (baseline)
    for (auto c : AcChrominanceCodesPerBitsize)
      output(c);
    for (auto c : AcChrominanceValues)
      output(c);

    // compute actual Huffman code tables (see Jon's code for precalculated tables)
    generateHuffmanTable(DcChrominanceCodesPerBitsize, DcChrominanceValues, huffmanChrominanceDC);
    generateHuffmanTable(AcChrominanceCodesPerBitsize, AcChrominanceValues, huffmanChrominanceAC);
  }

  // ////////////////////////////////////////
  // start of scan (there is only a single scan for baseline JPEGs)
  writeMarker(output, 0xDA, 2+1+2*numComponents+3);

  // assign Huffman tables to each component
  output(numComponents);
  for (uint8_t i = 1; i <= numComponents; i++)
  {
    // component ID
    output(i);
    // highest 4 bits: DC Huffman table, lowest 4 bits: AC Huffman table
    output(i == 1 ? 0x00 : 0x11); // Y: tables 0 for DC and AC; Cb + Cr: tables 1 for DC and AC
  }

  // constant values for our baseline JPEGs with a single sequential scan
  output( 0); // spectral selection: must start at  0
  output(63); // spectral selection: must stop  at 63
  output( 0); // successive approximation: must be  0

  // scaling constants for AAN DCT algorithm:
  // AanScaleFactors[0] = 1
  // AanScaleFactors[k] = cos(k*PI/16) * sqrt(2)    for k=1..7
  static const float AanScaleFactors[8] = { 1, 1.387039845f, SqrtHalfSqrt, 1.175875602f, 1, 0.785694958f, InvSqrtSqrt, 0.275899379f };
  float scaledLuminance  [64];
  float scaledChrominance[64];
  for (auto i = 0; i < 64; i++)
  {
    auto row    = i >> 3; // div 8
    auto column = i &  7; // mod 8

    auto factor = 1 / (AanScaleFactors[row] * AanScaleFactors[column] * 8);
    scaledLuminance  [i] = factor / quantLuminance  [ZigZag[i]];
    scaledChrominance[i] = factor / quantChrominance[ZigZag[i]];
  }

  // used to write bits to output
  BitBuffer buffer;

  // just convert image data from void*
  auto pixels = (const uint8_t*)pixels_;
  // convert from short to int to prevent overflows in calculating pixelPos
  int32_t height = height_;
  int32_t width  = width_;

  // downsampling of Cb and Cr channels
  uint8_t sampling = downSample ? 2 : 1;
  // basic 4:4:4 format ?
  bool isYCbCr444  = isRGB && !downSample;

  // process MCUs (minimum codes units)
  int16_t lastYDC = 0, lastCbDC = 0, lastCrDC = 0;
  for (auto mcuY = 0; mcuY < height; mcuY += 8 * sampling)
    for (auto mcuX = 0; mcuX < width; mcuX += 8 * sampling)
    {
      // break down the image into 8x8 blocks, convert from RGB or grayscale to YCbCr and then run JUPEG's compression algorithm
      float Y[8][8], Cb[8][8], Cr[8][8];

      // YCbCr 4:4:4 format: each MCU is a 8x8 block - the same applies to grayscale images, too
      // YCbCr 4:2:0 format: each MCU represents a 16x16 block, stored as 4x 8x8 Y-blocks plus 1x 8x8 Cb and 1x 8x8 Cr blocks)
      for (auto blockY = 0; blockY < 8 * sampling; blockY += 8) // these loops are iterated just once (grayscale, 4:4:4) or twice (4:2:0)
        for (auto blockX = 0; blockX < 8 * sampling; blockX += 8)
        {
          // now we finally have a 8x8 block ...
          for (auto deltaY = 0; deltaY < 8; deltaY++)
            for (auto deltaX = 0; deltaX < 8; deltaX++)
            {
              // find actual pixel position within the current image
              auto column   = clamp(mcuX + deltaX + blockX, 0, width  - 1); // must not exceed image borders, replicate last row/column if needed
              auto row      = clamp(mcuY + deltaY + blockY, 0, height - 1);
              // RGB: 3 bytes per pixel, grayscale: 1 byte per pixel
              auto pixelPos = (row * width + column) * numComponents; 

              // grayscale images have solely a Y channel which can be easily derived from the input pixel by shifting it by 128
              if (!isRGB)
              {
                Y[deltaY][deltaX] = pixels[pixelPos] - 128.f;
                continue;
              }

              auto r = pixels[pixelPos    ];
              auto g = pixels[pixelPos + 1];
              auto b = pixels[pixelPos + 2];

              // convert to YCbCr, constants are similar to ITU-R, see https://en.wikipedia.org/wiki/YCbCr#JPEG_conversion
              Y   [deltaY][deltaX] = +0.299f    * r +0.587f    * g +0.114f    * b - 128.f;

              if (isYCbCr444)
              {
                Cb[deltaY][deltaX] = -0.168736f * r -0.331264f * g +0.5f      * b;
                Cr[deltaY][deltaX] = +0.5f      * r -0.418688f * g -0.081312f * b;
              }
            }

          // encode Y channel
          lastYDC    = processDU(output, buffer, Y,  scaledLuminance,   lastYDC,  huffmanLuminanceDC,   huffmanLuminanceAC);

          // YCbCr 4:4:4 ? => encode Cb + Cr as well
          if (isYCbCr444)
          {
            lastCbDC = processDU(output, buffer, Cb, scaledChrominance, lastCbDC, huffmanChrominanceDC, huffmanChrominanceAC);
            lastCrDC = processDU(output, buffer, Cr, scaledChrominance, lastCrDC, huffmanChrominanceDC, huffmanChrominanceAC);
          }
        }

      // the following lines are only relevant for deferred downsampled Cb+Cr (that means YCbCr 4:2:0)
      if (!downSample)
        continue;

      // ////////////////////////////////////////
      // the following Cb+Cr code looks a bit more complicated because I have to average/downsample chrominance of four pixels
      for (auto deltaY = 0; deltaY < 8; deltaY++)
        for (auto deltaX = 0; deltaX < 8; deltaX++)
        {
          // a little bit different: scale deltas to take sampling into account (note: if you reach this line then sampling is always 2)
          auto column   = clamp(mcuX + sampling*deltaX, 0, width  - 1);
          auto row      = clamp(mcuY + sampling*deltaY, 0, height - 1);

          // the other three pixels must not exceed image borders
          // relative offsets, first entry isn't used
          int32_t offsets[] = {    0,     1,
                                width, width+1 };
          // right border ?
          if (column == width  - 1)
          {
            offsets[1]--;
            offsets[3]--;
          }
          // bottom border ?
          if (row    == height - 1)
          {
            offsets[2] -= width;
            offsets[3] -= width;
          }

          // let's add all four samples (computing their average is slightly deferred, see about 10 lines below)
          auto r = 0, g = 0, b = 0;
          auto numSamples = sampling * sampling;
          for (auto s = 0; s < numSamples; s++)
          {
            auto pixelPosSample = (row * width + column + offsets[s]) * numComponents;
            r += pixels[pixelPosSample    ];
            g += pixels[pixelPosSample + 1];
            b += pixels[pixelPosSample + 2];
          }

          // convert to YCbCr, constants are similar to ITU-R, see https://en.wikipedia.org/wiki/YCbCr#JPEG_conversion
          Cb[deltaY][deltaX] = (-0.168736f * r -0.331264f * g +0.5f      * b) / numSamples; // I deferred the division up to here for faster speed
          Cr[deltaY][deltaX] = (+0.5f      * r -0.418688f * g -0.081312f * b) / numSamples; // => just 2 divisions instead of 3 (for r,g,b)
        }

        // encode DUs (Cb + Cr channels)
        lastCbDC = processDU(output, buffer, Cb, scaledChrominance, lastCbDC, huffmanChrominanceDC, huffmanChrominanceAC);
        lastCrDC = processDU(output, buffer, Cr, scaledChrominance, lastCrDC, huffmanChrominanceDC, huffmanChrominanceAC);
      }

  // fill remaining bits with 1s
  writeBits(output, buffer, { 0x7F, 7 });

  // ///////////////////////////
  // EOI marker
  output(0xFF); output(0xD9);
  return true;
} // writeJpeg()

} // namespace TooJpeg
