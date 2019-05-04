# A JPEG encoder in a single C++ file

This is a mirror of my library hosted at https://create.stephan-brumme.com/toojpeg/

This is a compact baseline JPEG/JFIF writer, written in C++ (but looks like C for the most part).
Its interface has only one function: writeJpeg() - and that's it !

# How to use

1. create an image with any content you like, e.g. 1024x768, RGB (3 bytes per pixel)

```cpp
   auto pixels = new unsigned char[1024*768*3];
```

2. define a callback that receives the compressed data byte-by-byte 

```cpp
// for example, write to disk (could be anything else, too, such as network transfer, in-memory storage, etc.)
void myOutput(unsigned char oneByte) { fputc(oneByte, myFileHandle); }
```

3. start JPEG compression

```cpp
writeJpeg(myOutput, mypixels, 1024, 768);
// actually there are some optional parameters, too
//bool ok = tooJpeg(writeByte, pixels, width, height, imgType, quality, downSample, comment);
```
