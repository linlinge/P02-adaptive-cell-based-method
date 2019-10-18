#include "Color.h"
V3 get_color(float min,float max,float confidence)
{
        V3 color_temp;
        float n=4.0; //
        float step=(max-min)/n;
        float c=confidence-min;
        if(c<step)
        {
                color_temp.r=0;
                color_temp.g=c/step*255;
                color_temp.b=255;
        }
        else if(c<2*step)
        {
                color_temp.r=0;
                color_temp.g=255;
                color_temp.b=255-(c-step)/step*255;
        }
        else if(c<3*step)
        {
                color_temp.r=(c-2*step)/step*255;
                color_temp.g=255;
                color_temp.b=0;
        }
        else
        {
                color_temp.r=255;
                color_temp.g=255-(c-3*step)/step*255;
                color_temp.b=0;
        }

        return color_temp;
}