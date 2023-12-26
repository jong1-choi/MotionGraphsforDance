//
//  pfm.hpp
//  IK
//
//  Created by 최종원 on 2023/10/10.
//

#ifndef pfm_hpp
#define pfm_hpp

#include <stdio.h>

inline void savePFM(const std::string& fn, size_t w, size_t h, size_t c, float* data, bool flip = true ) {
   std::ofstream ofs( fn, std::ios::binary );
   if(c==1) ofs<<"Pf\n"<<w<<" "<<h<<"\n-1.0\n";
   if(c==3) ofs<<"PF\n"<<w<<" "<<h<<"\n-1.0\n";
   if( flip )
      for( int y=0; y<h; y++)
         ofs.write((char*)(data+w*c*(h-1-y)),w*c*sizeof(float));
   else
      for( int y=0; y<h; y++)
         ofs.write((char*)(data+w*c*(y)),w*c*sizeof(float));

   ofs.close();
}

inline std::tuple<int,int,int,float*> loadPFM( const std::string& fn ) {
   char buf[1024];
   int w=0, h=0, c=0;
   float endianness;
   
   std::ifstream file(fn, std::ios::binary);
   if(!file.is_open() ) throw "File not found";
   file.getline(buf, 1024);
   if( buf[0] != 'P' ) throw "Not supported file";
   if( buf[1] == 'F' ) c = 3;
   else c = 1;
   file >> w >> h >> endianness;
   file.getline(buf, 1024);
   
   float* data = new float[w*h*c];
   if( endianness<0 ) /*little endian <native> */ {
      for( int y=0; y<h; y++ ) {
         file.read((char*)(data+(h-1-y)*w*c), w*c*sizeof(float));
      }
   }
   else   /* Big endian <need conversion> */ {
      char bigBuf[4];
      char littleBuf[4];
      for( int y=0; y<h; y++ ) {
         for( int i=0; i<w*c; i++) {
            file.read(bigBuf,4);
            littleBuf[0] = bigBuf[3];
            littleBuf[1] = bigBuf[2];
            littleBuf[2] = bigBuf[1];
            littleBuf[3] = bigBuf[0];
            data[i+(h-1-y)*w*c] = *((float*)littleBuf);
         }
      }
   }
   
   file.close();
   return std::make_tuple(w,h,c,data);
}
#endif /* pfm_hpp */
