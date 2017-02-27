
//Generating 2d east asian stone chime instruments like Mesh model, procedually made variations
//from Korean PyeonGyeong to Chiniese Bianqing
//by Philip Liu, 2016

#include "StoneChime.h"
#include <cmath>
#include <algorithm>

using namespace std;

t_shape* calcMesh(int meshNum, float angle, int fragNums){

vector<t_shape> shapes;
vector<t_point> pArr;
t_point tmp;

const int coords[] = {0,0,
    -2,0,
    2,0,
    1,1,
    -1,1,
    -1,-1,
    1,-1,
};

int pgHeight = 6;
    

for(int k = -2; k<(pgHeight/2); k=k+2){
        for(int j = -pgHeight; j< pgHeight; j=j+2){
            for(int i=0; i<sizeof(coords)/sizeof(*coords)-1; i=i+2){
                tmp.x = coords[i]+k;
                tmp.y = coords[i+1]-j;
                pArr.push_back(tmp);
            }
        }
}


for(int k = -2; k<(pgHeight/2); k=k+2){
        for(float c = 0; c<(3-(angle/12)+(pgHeight/4)); c=c+(2/angle)){
            for(int i=0; i<sizeof(coords)/sizeof(*coords)-1; i=i+2){
                tmp.x = coords[i]+2+k+(c*(angle/2));
                tmp.y = coords[i+1]-pgHeight-c;
                pArr.push_back(tmp);

                tmp.x = coords[i]+2+k+(c*(angle/2))-1;
                tmp.y = coords[i+1]-pgHeight-c-1;
                pArr.push_back(tmp);
            }
        }
}



t_point* p = &pArr[0];
shapes.push_back(*getShape2(0, p, 1));
shapes.push_back(*getShape2(1, p, 1));
shapes.push_back(*getShape2(0, p, pArr.size()));



pArr.clear();

    
    //Manually made fragment mesh model of the first stone chime instrument above
    
int drawCoords[] = {0,-6,0,-4,0,-2,0,0,0,2,0,4,0,6,
                   2,-6,4,-6,6,-8,4,-8,2,-8,2,-4, 2,-6,4,-4,2,2,4,0,4,2,4,4,4,6,4,-2,
                   4,-10, 6, -10,8,-10};

if(fragNums > 47){
    fragNums = 47;
}

for(int i=0; i<sizeof(coords)/sizeof(*coords)-1; i=i+2){

    for(int j=0; j<fragNums; j=j+2){

        tmp.x = coords[i]+drawCoords[j];
        tmp.y = coords[i+1]+drawCoords[j+1];
        pArr.push_back(tmp);
    }
}

p = &pArr[0];

shapes.push_back(*getShape2(0, p, pArr.size()));


    
    return &shapes[meshNum];

}
