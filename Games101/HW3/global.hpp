//
// Created by LEI XU on 4/9/19.
//

#ifndef RASTERIZER_GLOBAL_H
#define RASTERIZER_GLOBAL_H

typedef unsigned char u08;
#define MY_PI 3.1415926
#define TWO_PI (2.0* MY_PI)

#define Min(x,y) x>y? y:x
#define Max(x,y) x<y? y:x
#define Clamp(min,max,value) Min(Max(min,value),max)


#endif //RASTERIZER_GLOBAL_H
