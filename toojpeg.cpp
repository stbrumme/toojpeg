// //////////////////////////////////////////////////////////
// toojpeg.cpp
// written by Stephan Brumme, 2018-2019
// see https://create.stephan-brumme.com/toojpeg/
//

#include "toojpeg.h"

// notes:
// - the "official" specifications: https://www.w3.org/Graphics/JPEG/itu-t81.pdf and https://www.w3.org/Graphics/JPEG/jfif3.pdf
// - a short documentation of the JFIF/JPEG file format can be found in the Wikipedia: https://en.wikipedia.org/wiki/JPEG_File_Interchange_Format
// - the popular STB Image library includes Jon's JPEG encoder as well: https://github.com/nothings/stb/blob/master/stb_image_write.h
// - the most readable JPEG book (from a developer's perspective) is Miano's "Compressed Image File Formats" (1999, ISBN 0-201-60443-4),
//   used copies are really cheap nowadays and it includes a CD with C++ sources as well (plus detailled format descriptions of GIF+PNG)
// - much more detailled is Mitchell/Pennebaker's "JPEG: Still Image Data Compression Standard" (1993, ISBN 0-442-01272-1)
//   which contains the official JPEG standard, too - fun fact: I bought a signed copy in a second-hand store without noticing

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

// represent a few bits, typically a Huffman code
struct BitCode
{
  inline BitCode(uint16_t code_ = 0, uint8_t numBits_ = 0)
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
const auto BlockSize = 8 * 8; // = 64
// most encoders use a zig-zag table, I switched to its inverse for performance reasons
// note: ZigZagInv[ZigZag[i]] = i
static const uint8_t ZigZagInv[BlockSize] =
    {  0, 1, 8,16, 9, 2, 3,10,   // ZigZag[] =  0, 1, 5, 6,14,15,27,28,
      17,24,32,25,18,11, 4, 5,   //             2, 4, 7,13,16,26,29,42,
      12,19,26,33,40,48,41,34,   //             3, 8,12,17,25,30,41,43,
      27,20,13, 6, 7,14,21,28,   //             9,11,18,24,31,40,44,53,
      35,42,49,56,57,50,43,36,   //            10,19,23,32,39,45,52,54,
      29,22,15,23,30,37,44,51,   //            20,22,33,38,46,51,55,60,
      58,59,52,45,38,31,39,46,   //            21,34,37,47,50,56,59,61,
      53,60,61,54,47,55,62,63 }; //            35,36,48,49,57,58,62,63
// some constants for our DCT
const auto SqrtHalfSqrt = 1.306562965f; // sqrt((2 + sqrt(2)) / 2)  = cos(pi * 1 / 8) * sqrt(2)
const auto HalfSqrtSqrt = 0.382683433f; // sqrt( 2 - sqrt(2)) / 2   = cos(pi * 3 / 8)
const auto InvSqrt      = 0.707106781f; // 1 / sqrt(2)              = cos(pi * 2 / 8)
const auto InvSqrtSqrt  = 0.541196100f; // 1 / sqrt(2 - sqrt(2))    = cos(pi * 3 / 8) * sqrt(2)

// ////////////////////////////////////////
// helper functions

// restrict a value to the interval [minimum, maximum]
template <typename Number, typename Limit>
inline Number clamp(Number value, Limit minimum, Limit maximum)
{
  if (value <= minimum) return minimum; // never smaller than the minimum
  if (value >= maximum) return maximum; // never bigger  than the maximum
  return value;                         // value was inside interval, keep it
}

// start a new JFIF block
inline void writeMarker(TooJpeg::WRITE_ONE_BYTE output, uint8_t id, uint16_t length)
{
  output(0xFF); output(id);      // ID, always preceded by 0xFF
  output(uint8_t(length >> 8));  // length (big-endian)
  output(uint8_t(length & 0xFF));
}

// write bits stored in BitCode, keep excess bits in BitBuffer
inline void writeBits(TooJpeg::WRITE_ONE_BYTE output, BitBuffer& buffer, BitCode data)
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
    auto oneByte = (buffer.bits >> buffer.numBits) & 0xFF;
    output(oneByte);

    if (oneByte == 0xFF) // 0xFF has a special meaning for JPEGs (it's a block marker)
      output(0);         // therefore pad a zero to indicate "nope, this one ain't a marker, it's just a coincidence"

    // note: I don't clear those written bits, therefore buffer.bits contains garbage in the high bits
    //       if you really want to "clean up" (e.g. for debugging purposes) then uncomment the following line
    //buffer.bits &= (1 << buffer.numBits) - 1;
  }
}

// convert to JPEG bit code
inline BitCode convertCode(int16_t value)
{
  // positive value: code = value,     numBits = position of highest set bit
  // negative value: ignore sign, then numBits = position of highest set bit, and code = (2^numBits) - 1 + value
  BitCode result(value, 0);

  auto absolute = value < 0 ? -value : +value; // by the way: value is never zero
  auto mask = 0; // will be 2^numBits - 1
  // find position of highest set bit, fast way for GCC: result.numBits = 32 - __builtin_clz(value);
  while (absolute > mask)
  {
    result.numBits++;
    mask = 2*mask + 1;   // append a set bit (numBits increased by one, so we need to update 2^numBits - 1)
  }

  if (value < 0)
    result.code += mask; // remember: mask = 2^numBits - 1
  return result;
}

// forward DCT computation (fast AAN algorithm: Arai, Agui and Nakajima: "A fast DCT-SQ scheme for images")
inline void DCT(float* block, uint8_t stride) // stride = 1 or 8 (horizontal or vertical)
{
  // modify in-place
  auto& block0 = block[0         ]; // same as 0 * stride
  auto& block1 = block[    stride]; // same as 1 * stride
  auto& block2 = block[2 * stride];
  auto& block3 = block[3 * stride];
  auto& block4 = block[4 * stride];
  auto& block5 = block[5 * stride];
  auto& block6 = block[6 * stride];
  auto& block7 = block[7 * stride];

  // based on https://dev.w3.org/Amaya/libjpeg/jfdctflt.c , the original variable names can be found in my comments
  auto add07 = block0 + block7; auto sub07 = block0 - block7; // tmp0, tmp7
  auto add16 = block1 + block6; auto sub16 = block1 - block6; // tmp1, tmp6
  auto add25 = block2 + block5; auto sub25 = block2 - block5; // tmp2, tmp5
  auto add34 = block3 + block4; auto sub34 = block3 - block4; // tmp3, tmp4

  auto add0347 = add07 + add34; auto sub07_34 = add07 - add34; // tmp10, tmp13 ("even part" / "phase 2")
  auto add1256 = add16 + add25; auto sub16_25 = add16 - add25; // tmp11, tmp12

  block0 = add0347 + add1256; block4 = add0347 - add1256; // "phase 3"

  auto z1 = (sub16_25 + sub07_34) * InvSqrt; // all temporary z-variables kept their original names
  block2 = sub07_34 + z1; block6 = sub07_34 - z1; // "phase 5"

  auto sub23_45 = sub25 + sub34; // tmp10 ("odd part" / "phase 2")
  auto sub12_56 = sub16 + sub25; // tmp11
  auto sub01_67 = sub16 + sub07; // tmp12

  auto z5 = (sub23_45 - sub01_67) * HalfSqrtSqrt;
  auto z2 = sub23_45 * InvSqrtSqrt  + z5;
  auto z3 = sub12_56 * InvSqrt;
  auto z4 = sub01_67 * SqrtHalfSqrt + z5;
  auto z6 = sub07 + z3; // z11 ("phase 5")
  auto z7 = sub07 - z3; // z13
  block1 = z6 + z4; block7 = z6 - z4; // "phase 6"
  block5 = z7 + z2; block3 = z7 - z2;
}

// process 8x8 block
int16_t processDU(TooJpeg::WRITE_ONE_BYTE output, BitBuffer& buffer,
                  float block[8][8], const float scaled[BlockSize], int16_t lastDC,
                  const BitCode huffmanDC[256], const BitCode huffmanAC[256])
{
  // "linearize" the 8x8 block, treat it as a flat array of 64 floats
  auto block64 = (float*) block;

  // DCT: rows
  for (auto offset = 0; offset < 8; offset++)
    DCT(block64 + offset*8, 1);
  // DCT: columns
  for (auto offset = 0; offset < 8; offset++)
    DCT(block64 + offset*1, 8);

  // scale
  for (auto i = 0; i < BlockSize; i++)
    block64[i] *= scaled[i];

  // encode DC (the first coefficient is the "average color" of the 8x8 block)
  // convert to an integer
  int16_t DC = block64[0] + (block64[0] > 0 ? +0.5f : -0.5f); // C++11's nearbyint() achieves a similar effect
  // same "average color" as previous block ?
  if (DC == lastDC)
    writeBits(output, buffer, huffmanDC[0x00]); // yes, write a special short symbol
  else
  {
    auto bits = convertCode(DC - lastDC);       // encode the difference to previous block's average color
    writeBits(output, buffer, huffmanDC[bits.numBits]);
    writeBits(output, buffer, bits);
  }

  // quantize and zigzag the other 63 coefficients
  auto posNonZero = 0; // find last coefficient which is not zero (because trailing zeros are encoded very efficiently)
  int16_t quantized[BlockSize];
  for (auto i = 1; i < BlockSize; i++)
  {
    auto value = block64[ZigZagInv[i]];
    // round to nearest integer (actually, rounding is performed by casting from float to int16)
    quantized[i] = value + (value > 0 ? +0.5f : -0.5f); // C++11's nearbyint() achieves a similar effect
    // remember offset of last non-zero coefficient
    if (quantized[i] != 0)
      posNonZero = i;
  }

  // encode ACs (Q[1..63])
  for (auto i = 1; i <= posNonZero; i++) // Q[0] was already written, start at Q[1] and skip all trailing zeros
  {
    // zeros are encoded in a special way
    auto offset = 0;  // upper 4 bits count the number of consecutive zeros
    while (quantized[i] == 0) // found a few zeros, let's count them
    {
      i++;
      offset     +=  1 << 4; // add 1 to the upper 4 bits
      // split into blocks of at most 16 consecutive zeros
      if (offset == 16 << 4)
      {
        writeBits(output, buffer, huffmanAC[0xF0]); // 0xF0 is a special code for "16 zeros"
        offset = 0;
      }
    }

    // merge number of remaining zeros and next non-zero value
    auto bits = convertCode(quantized[i]);
    offset   += bits.numBits;
    writeBits(output, buffer, huffmanAC[offset]);
    writeBits(output, buffer, bits);
  }

  // send end-of-block code (0x00), only needed if there are trailing zeros
  if (posNonZero < BlockSize - 1)
    writeBits(output, buffer, huffmanAC[0x00]);

  return DC;
}

// Jon's code includes the pre-generated Huffman codes
// I don't like these "magic constants" and compute them on my own :-)
void generateHuffmanTable(const uint8_t numCodes[16], const uint8_t* values, BitCode result[256])
{
  uint16_t huffmanCode = 0; // no JPEG Huffman code exceeds 16 bits
  // process all bitsizes 1-16 ...
  for (auto numBits = 1; numBits <= 16; numBits++)
  {
    // ... and each code of these bitsizes
    for (auto i = 0; i < numCodes[numBits - 1]; i++) // note: numCodes array starts at zero, but smallest bitsize is 1
    {
      auto current = *values++;
      result[current].code    = huffmanCode++;
      result[current].numBits = numBits;
    }
    // next Huffman code needs to be one bit wider
    huffmanCode <<= 1;
  }
}

// convert from RGB to YCbCr, constants are similar to ITU-R, see https://en.wikipedia.org/wiki/YCbCr#JPEG_conversion
template <typename T>
float rgb2y (T r, T g, T b) { return +0.299f   * r +0.587f    * g +0.114f  * b; }
template <typename T>
float rgb2cb(T r, T g, T b) { return -0.16874f * r -0.33126f * g +0.5f     * b; } // ITU: -0.168736f * r -0.331264f * g +0.5f      * b
template <typename T>
float rgb2cr(T r, T g, T b) { return +0.5f     * r -0.41869f * g -0.08131f * b; } // ITU: +0.5f      * r -0.418688f * g -0.081312f * b

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
  uint16_t quality = clamp(quality_, 1, 100);
  // formula taken from libjpeg
  quality = quality < 50 ? 5000 / quality : 200 - quality * 2;

  // number of components
  const auto numComponents = isRGB ? 3 : 1;
  // note: if there is just one component (=grayscale), then only luminance needs to be stored in the file
  //       thus everything related to chrominance need not to be written to the JPEG
  //       I still compute a few things, like quantization tables to avoid a complete code mess

  // grayscale images can't be downsampled (because there are no Cb + Cr channels)
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
        0,1,0,1,           // density: 1 pixel "per pixel" horizontally and vertically
        0,0 };             // no thumbnail (size 0 x 0)
  for (auto c : HeaderJfif)
    output(c);

  // ////////////////////////////////////////
  // comment (optional)
  if (comment != 0)
  {
    // look for zero terminator
    uint16_t length = 0; // = strlen(comment);
    while (comment[length] != 0)
      length++;

    // length: number of bytes (without zero terminator) + 2 bytes for this length field
    writeMarker(output, 0xFE, length + 2); // COM marker
    // ... and the comment itself
    for (auto i = 0; i < length; i++)
      output(comment[i]);
  }

  // ////////////////////////////////////////
  // quantization tables from JPEG Standard, Annex K
  // there are a few experts proposing slightly more efficient values, e.g. https://www.imagemagick.org/discourse-server/viewtopic.php?t=20333
  // btw: Google's Guetzli project attempts to optimize these tables per image
  static const uint8_t DefaultQuantLuminance[BlockSize] =
                     { 16, 11, 10, 16, 24, 40, 51, 61,
                       12, 12, 14, 19, 26, 58, 60, 55,
                       14, 13, 16, 24, 40, 57, 69, 56,
                       14, 17, 22, 29, 51, 87, 80, 62,
                       18, 22, 37, 56, 68,109,103, 77,
                       24, 35, 55, 64, 81,104,113, 92,
                       49, 64, 78, 87,103,121,120,101,
                       72, 92, 95, 98,112,100,103, 99 };
  static const uint8_t DefaultQuantChrominance[BlockSize] =
                     { 17, 18, 24, 47, 99, 99, 99, 99,
                       18, 21, 26, 66, 99, 99, 99, 99,
                       24, 26, 56, 99, 99, 99, 99, 99,
                       47, 66, 99, 99, 99, 99, 99, 99,
                       99, 99, 99, 99, 99, 99, 99, 99,
                       99, 99, 99, 99, 99, 99, 99, 99,
                       99, 99, 99, 99, 99, 99, 99, 99,
                       99, 99, 99, 99, 99, 99, 99, 99 };

  // adjust quantization tables to desired quality
  uint8_t quantLuminance  [BlockSize];
  uint8_t quantChrominance[BlockSize];
  for (auto i = 0; i < BlockSize; i++)
  {
    uint16_t luminance   = (DefaultQuantLuminance  [ZigZagInv[i]] * quality + 50) / 100;
    uint16_t chrominance = (DefaultQuantChrominance[ZigZagInv[i]] * quality + 50) / 100;

    // clamp to 1..255
    quantLuminance  [i] = uint8_t(clamp(luminance,   1, 255));
    quantChrominance[i] = uint8_t(clamp(chrominance, 1, 255));
  }

  // write quantization tables
  writeMarker(output, 0xDB, 2 + (isRGB ? 2 : 1) * (1+BlockSize)); // length: 65 bytes per table + 2 bytes for this length field
                                                                  // each table has 64 entries and is preceded by an ID byte

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
  for (auto i = 1; i <= numComponents; i++)
  {
    output(i);                    // component ID (Y=1, Cb=2, Cr=3)
    // bitmasks for sampling: highest 4 bits: horizontal, lowest 4 bits: vertical
    output(i == 1 && downSample ? 0x22 : 0x11); // 0x11 is default YCbCr 4:4:4 and 0x22 stands for YCbCr 4:2:0
    output(i == 1 ? 0 : 1);       // use quantization table 0 for Y, else table 1
  }

  // ////////////////////////////////////////
  // Huffman tables
  BitCode huffmanLuminanceDC  [256];
  BitCode huffmanLuminanceAC  [256];
  BitCode huffmanChrominanceDC[256];
  BitCode huffmanChrominanceAC[256];

  // Huffman definitions for first DC table
  static const uint8_t DcLuminanceCodesPerBitsize[16] = { 0,1,5,1,1,1,1,1,1,0,0,0,0,0,0,0 };   // sum = 12
  static const uint8_t DcLuminanceValues         [12] = { 0,1,2,3,4,5,6,7,8,9,10,11 };         // => 12 codes
  // Huffman definitions for first AC table
  static const uint8_t AcLuminanceCodesPerBitsize[16] = { 0,2,1,3,3,2,4,3,5,5,4,4,0,0,1,125 }; // sum = 162
  static const uint8_t AcLuminanceValues        [162] =                                        // => 162 codes
      { 0x01,0x02,0x03,0x00,0x04,0x11,0x05,0x12,0x21,0x31,0x41,0x06,0x13,0x51,0x61,0x07,0x22,0x71,0x14,0x32,0x81,0x91,0xA1,0x08,
        0x23,0x42,0xB1,0xC1,0x15,0x52,0xD1,0xF0,0x24,0x33,0x62,0x72,0x82,0x09,0x0A,0x16,0x17,0x18,0x19,0x1A,0x25,0x26,0x27,0x28,
        0x29,0x2A,0x34,0x35,0x36,0x37,0x38,0x39,0x3A,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x53,0x54,0x55,0x56,0x57,0x58,0x59,
        0x5A,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6A,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7A,0x83,0x84,0x85,0x86,0x87,0x88,0x89,
        0x8A,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9A,0xA2,0xA3,0xA4,0xA5,0xA6,0xA7,0xA8,0xA9,0xAA,0xB2,0xB3,0xB4,0xB5,0xB6,
        0xB7,0xB8,0xB9,0xBA,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7,0xC8,0xC9,0xCA,0xD2,0xD3,0xD4,0xD5,0xD6,0xD7,0xD8,0xD9,0xDA,0xE1,0xE2,
        0xE3,0xE4,0xE5,0xE6,0xE7,0xE8,0xE9,0xEA,0xF1,0xF2,0xF3,0xF4,0xF5,0xF6,0xF7,0xF8,0xF9,0xFA };

  // DHT marker - define Huffman tables
  writeMarker(output, 0xC4, isRGB ? (2+208+208) : (2+208));
                                  // 2 bytes for the length field, store chrominance only if needed
                                  //   1+16+12  for the DC luminance
                                  //   1+16+162 for the AC luminance   (208 = 1+16+12 + 1+16+162)
                                  //   1+16+12  for the DC chrominance
                                  //   1+16+162 for the AC chrominance (208 = 1+16+12 + 1+16+162, same as above)

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
    const auto& DcChrominanceValues = DcLuminanceValues;//= { 0,1,2,3,4,5,6,7,8,9,10,11 };       // (identical to DcLuminanceValues)
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
  for (auto i = 1; i <= numComponents; i++)
  {
    // component ID (Y=1, Cb=2, Cr=3)
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
  float scaledLuminance  [BlockSize];
  float scaledChrominance[BlockSize];
  for (auto i = 0; i < BlockSize; i++)
  {
    auto row    = ZigZagInv[i] / 8; // same as i >> 3
    auto column = ZigZagInv[i] % 8; // same as i &  7
    auto factor = 1 / (AanScaleFactors[row] * AanScaleFactors[column] * 8);
    scaledLuminance  [ZigZagInv[i]] = factor / quantLuminance  [i];
    scaledChrominance[ZigZagInv[i]] = factor / quantChrominance[i];
    // if you really want JPEGs that are bitwise identical to Jon's code then you need slightly different formulas (note: sqrt(8) = 2.828427125f)
    //static const float aasf[] = { 1.0f * 2.828427125f, 1.387039845f * 2.828427125f, 1.306562965f * 2.828427125f, 1.175875602f * 2.828427125f, 1.0f * 2.828427125f, 0.785694958f * 2.828427125f, 0.541196100f * 2.828427125f, 0.275899379f * 2.828427125f }; // line 240 of jo_jpeg.cpp
    //scaledLuminance  [ZigZagInv[i]] = 1 / (quantLuminance  [i] * aasf[row] * aasf[column]); // lines 266-267 of jo_jpeg.cpp
    //scaledChrominance[ZigZagInv[i]] = 1 / (quantChrominance[i] * aasf[row] * aasf[column]);
  }

  // all encoded bits pass through this buffer, it writes to output whenever a byte is completed
  BitBuffer buffer;

  // just convert image data from void*
  auto pixels = (const uint8_t*)pixels_;
  // convert from short to int to prevent overflows in calculating pixelPos
  int32_t height  = height_;
  int32_t width   = width_;
  // downsampling of Cb and Cr channels, if sampling = 2 then 2x2 samples are used
  auto sampling   = downSample ? 2 : 1;
  // basic 4:4:4 format ?
  bool isYCbCr444 = isRGB && !downSample;

  // process MCUs (minimum codes units)
  int16_t lastYDC = 0, lastCbDC = 0, lastCrDC = 0;
  for (auto mcuY = 0; mcuY < height; mcuY += 8 * sampling)
    for (auto mcuX = 0; mcuX < width; mcuX += 8 * sampling)
    {
      // break down the image into 8x8 blocks, convert from RGB or grayscale to YCbCr and then run JPEG's compression algorithm
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
              auto pixelPos = row * width + column;

              // grayscale images have solely a Y channel which can be easily derived from the input pixel by shifting it by 128
              if (!isRGB)
              {
                Y[deltaY][deltaX] = float(pixels[pixelPos]) - 128.f;
                continue;
              }

              // RGB: 3 bytes per pixel (whereas grayscale images have only 1 byte per pixel)
              pixelPos *= numComponents;
              auto r = pixels[pixelPos    ];
              auto g = pixels[pixelPos + 1];
              auto b = pixels[pixelPos + 2];

              Y   [deltaY][deltaX] = rgb2y (r, g, b) - 128; // again, the JPEG standard requires Y to be shifted by 128
              if (isYCbCr444)
              {
                Cb[deltaY][deltaX] = rgb2cb(r, g, b);
                Cr[deltaY][deltaX] = rgb2cr(r, g, b);
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
                                width, width+1 }; // one step down = a whole line = "width" pixels
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

          // let's add all four samples (computing their average is part of the RGB=>YCbCr conversion for performance reasons)
          auto r = 0, g = 0, b = 0;
          auto numSamples = sampling * sampling; // = 2*2 = 4
          for (auto s = 0; s < numSamples; s++)
          {
            auto pixelPosSample = (row * width + column + offsets[s]) * numComponents;
            r += pixels[pixelPosSample    ];
            g += pixels[pixelPosSample + 1];
            b += pixels[pixelPosSample + 2];
          }

          Cb[deltaY][deltaX] = rgb2cb(r, g, b) / numSamples;
          Cr[deltaY][deltaX] = rgb2cr(r, g, b) / numSamples;
        }

      // encode DUs (Cb + Cr channels)
      lastCbDC = processDU(output, buffer, Cb, scaledChrominance, lastCbDC, huffmanChrominanceDC, huffmanChrominanceAC);
      lastCrDC = processDU(output, buffer, Cr, scaledChrominance, lastCrDC, huffmanChrominanceDC, huffmanChrominanceAC);
    }

  // fill remaining bits with 1s
  writeBits(output, buffer, { 0x7F, 7 }); // seven set bits: 0x7F = binary 0111 1111

  // ///////////////////////////
  // EOI marker
  output(0xFF); output(0xD9);
  return true;
} // writeJpeg()

} // namespace TooJpeg
