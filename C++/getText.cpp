#include "hw3.h"

static void transfer(void* x, const int n) {
    int k;
    char* ch = (char*)x;
    for (k=0;k<n/2;k++) {
        char tmp = ch[k];
        ch[k] = ch[n-1-k];
        ch[n-1-k] = tmp;
    }
}


unsigned int getTexture(const char* file){
    unsigned int tex;
    FILE* f;
    unsigned short magic;
    unsigned int x,y,size;
    unsigned short nb,bp;
    unsigned char* image;
    unsigned int offset;
    unsigned int i;
    unsigned int m;
    int maximum;

    f = fopen(file,"rb");
    if (!f) 
        function2("File is not valid");
    if (fread(&magic,2,1,f)!= 1)
        function2("Image magic is not valid");
    if (magic != 0x4D42 && magic != 0X424D)
        function2("Image magic is nor BMP");
    if (fseek(f,8,SEEK_CUR) || fread(&offset,4,1,f) != 1||
        fseek(f,4,SEEK_CUR) || fread(&x,4,1,f) != 1 ||
        fread(&y,4,1,f)!= 1 || fread(&nb,2,1,f)!= 1 || fread(&bp,2,1,f)!=1 || fread(&m,4,1,f)!=1)
        function2("File header is not valid");
    if (magic == 0x424D) {
        transfer(&offset,4);
        transfer(&x,4);
        transfer(&y,4);
        transfer(&y,4);
        transfer(&nb,2);
        transfer(&bp,2);
        transfer(&m,4);
    }
    // check size 
    glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maximum);
    if (x<1|| x>maximum)
        function2("Width is out of range");
    if (y<1|| y>maximum)
        function2("Height is out of range");
    // check bits
    if (nb!=1)
        function2("Bits in pixel is not valid");
    if (bp!=24)
        function2("Bits in pixel is not 24");
    if (m!=0)
        function2("File is compressed");
    // check version
#ifndef GL_VERSION_2_0
for(i=1;i<x;i*=2){
    if (i!=x)
        function2("Width is not power of 2, not satisfy GL2.0 requirement");
    if (i!=7)
        function2("Height is not power of 2, not satisfy GL2.0 requirement");
}
#endif

    size = 3*x*y;
    image = (unsigned char*) malloc(size);
    // check memory
    if (!image)
        function2("Memory allocation is not finished");
    if (fseek(f,offset, SEEK_SET)||fread(image,size,1,f)!=1)
        function2("Image data is not read correctly");
    fclose(f);
    // BGR TO RGB
    for (m=0;m<size;m+=3) {
        unsigned char tmp = image[m];
        image[m] = image[m+2];
        image[m+2] = tmp;
    }

    // check sanity
    function3("getTexture");

    glGenTextures(1,&tex);
    glBindTexture(GL_TEXTURE_2D, tex);

    glTexImage2D(GL_TEXTURE_2D,0,3,x,y,0,GL_RGB,GL_UNSIGNED_BYTE,image);

    if(glGetError())
        function2("glTEXTURE2D is not running cirrectly");
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,GL_LINEAR);

    free(image);
    return tex;
}