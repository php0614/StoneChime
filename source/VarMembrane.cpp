//Modification of Membrane.cpp by Alex McLean (c) 2008, included in the Membrane UGen in SC3Plugins

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "SC_PlugIn.h"
#include "assert.h"

#include "StoneChime.h"

// twiddle-ables
#define SHAPE_SZ 16 // diameter
//#define FEEDFORWARD
#define SELF_LOOP        // for control over tension
#define RIMGUIDES // extra self-loops around edge, which invert signal //엣지 룹
#define RIMFILTER
#define AUDIO_INPUT
#define TRIGGER_DURATION 1024 /* number of samples worth of white noise to inject */

// some constants from Brook Eaton's roto-drum
// http://www-ccrma.stanford.edu/~be/drum/drum.htm

#define DELTA 6.0f // distance between junctions
#define GAMMA 8.0f // wave speed

// A unit delay
typedef struct {
  float a;
  float b;
  float c;
  int invert; // whether signal should be inverted
} t_delay;

// a scatter junction with up to 6 input and output delays, plus a
// self loop
typedef struct {
  int ins, outs;

  t_delay *in[6];
  t_delay *out[6];
#ifdef SELF_LOOP
  t_delay *self_loop;
#endif
} t_junction;


// supercollider stuff starts here...

// InterfaceTable contains pointers to functions in the host (server).
static InterfaceTable *ft;

// declare struct to hold unit generator state
struct VarMembrane : public Unit
{
  float yj; // junction admittence 교차로 입장, calculated from tension parameter 
#ifndef AUDIO_INPUT
  int triggered; // flag
  int excite;    // number of samples left in a triggered excitation
#endif
  t_shape *shape;
  t_junction *junctions;
  t_delay *delays;
  float loss;
  int delay_n;   // number of delays in mesh including self loops etc
};

// declare unit generator functions
extern "C"
{
  void VarMembrane_next_a(VarMembrane *unit, int inNumSamples);
  void VarMembraneCircle_Ctor(VarMembrane* unit);
  void VarMembraneHexagon_Ctor(VarMembrane* unit);
  void VarMembranePyeonGyeong_Ctor(VarMembrane* unit);
  void VarMembrane_Dtor(VarMembrane* unit);
};

////////////////////////////////////////////////////////////////////

// execute one sample cycle over the mesh

float cycle(VarMembrane *unit, float input, float yj_r) {
  //유닛에 딜레이를 읽어와 포인터를 만들어준다
  t_delay *delays = unit->delays;
  //역시 단순한 포인터
  t_junction *junctions = unit->junctions;

  int i;

  int middle = (int) (unit->shape->points_n / 2);
  float result;

  for (i = 0; i < unit->shape->points_n; ++i) {

    t_junction *junction = &junctions[i];
    int j;
    float total = 0;

    float yc = unit->yj - junction->ins;

    for (j = 0; j < junction->ins; ++j) {
      total += junction->in[j]->b;
    }

#ifdef SELF_LOOP
    total = 2.0f * (total + (yc * junction->self_loop->b)) * yj_r;
#else
    total *= (2.0f / ((float) junction->ins));
#endif

    if (i < middle) {
      total += (input / middle);
    }

    total *= unit->loss;

    for (j = 0; j < junction->outs; ++j) {
      junction->out[j]->a = total - junction->in[j]->b;
    }
#ifdef SELF_LOOP
    junction->self_loop->a = total - junction->self_loop->b;
#endif

    if (i == 0) {
      result = total;
    }
  }

  // circulate the unit delays
  for (i = 0; i < unit->delay_n; ++i) {
    t_delay *delay = &delays[i];
    if (delay->invert) {
#ifdef RIMFILTER
      delay->b = ((0.0f - delay->a) + delay->c) * 0.5f;
      delay->c = (0.0f - delay->a);
#else
      delay->b = 0.f - delay->a;
#endif
    }
    else {
      delay->b = delay->a;
    }
  }
  return(result);
}

////////////////////////////////////////////////////////////////////

void VarMembrane_init(VarMembrane* unit, int shape_type, int angle, int fragNums)
{

  t_shape *shape;
  int d = 0;
  int i = 0;
  int j = 0;

  SETCALC(VarMembrane_next_a);


#ifndef AUDIO_INPUT

  unit->triggered = 0;
  unit->excite = 0;
#endif

  unit->yj = 0;


    
    shape = unit->shape = calcMesh(shape_type, angle, fragNums);


  unit->delay_n = (shape->lines_n * 2)
#ifdef RIMGUIDES
    + shape->edge_n
#endif
#ifdef SELF_LOOP
    + shape->points_n
#endif
    ;


  unit->delays =

    (t_delay *) RTAlloc(unit->mWorld, unit->delay_n * sizeof(t_delay));

  memset((void *) unit->delays, 0, unit->delay_n * sizeof(t_delay));


  unit->junctions =
    (t_junction *) RTAlloc(unit->mWorld,
			   unit->shape->points_n * sizeof(t_junction)
			   );

  memset((void *) unit->junctions, 0,
	 unit->shape->points_n * sizeof(t_junction)
	 );



  for (i = 0; i < shape->lines_n; ++i) {

    t_line *line = shape->lines[i];

    t_junction *from, *to;

    t_delay *delay;

    from = &unit->junctions[line->a->id];
    to = &unit->junctions[line->b->id];

    delay = &unit->delays[d++];

    from->out[from->outs++] = delay;
    to->in[to->ins++] = delay;

    // rightward delay
    delay = &unit->delays[d++];
    from->in[from->ins++] = delay;
    to->out[to->outs++] = delay;
  }

  for (i = 0; i < shape->points_n; ++i) {
    t_point *point = shape->points[i];
    t_junction *junction = &unit->junctions[i];

#ifdef SELF_LOOP
    t_delay *delay = &unit->delays[d++];
    junction->self_loop = delay;
#endif

#ifdef RIMGUIDES

    assert((junction->ins < 6) == point->is_edge);
    if (point->is_edge) {
      t_delay *delay = &unit->delays[d++];
      delay->invert = 1;
      junction->out[junction->outs++] = delay;
      junction->in[junction->ins++] = delay;
    }
#endif
  }

  if(unit->mWorld->mVerbosity > 0){
    printf("%d delays initialised.\n", unit->delay_n);
  }

  // 3. Calculate one sample of output.
  // (why do this?)
  VarMembrane_next_a(unit, 1);
}


////////////////////////////////////////////////////////////////////

void VarMembrane_next_a(VarMembrane *unit, int inNumSamples) {
  // get the pointer to the output buffer
  float *out = OUT(0);
  int input_n = 0;
  // get the control rate input
#ifdef AUDIO_INPUT
  float *in = IN(input_n++);
#else
  float trigger = IN0(input_n++);
#endif

  float tension = IN0(input_n++);
  float loss = IN0(input_n++);

  if (tension == 0) {
    // default tension
    tension =  0.0001;
  }

  unit->yj = 2.f * DELTA * DELTA / (tension * tension * GAMMA * GAMMA);

  float yj_r = 1.0f / unit->yj;

  if (loss >= 1) {
    loss = 0.99999;
  }
  unit->loss = loss;

 
#ifndef AUDIO_INPUT
  if (trigger >= 0.5 && (! unit->triggered)) {

    unit->triggered = 1;
    unit->excite = TRIGGER_DURATION;

  }
  else if (trigger < 0.5 && unit->triggered) {
    unit->triggered = 0;
  }
#endif
  ////////////////////

  for (int k=0; k < inNumSamples; ++k) {
    float input = 0.0;
#ifdef AUDIO_INPUT

    input = in[k];
#else
    if (unit->excite > 0) {
      input = (0.01 - (((float) rand() / RAND_MAX) * 0.02));
      unit->excite--;
    }
#endif

    out[k] = cycle(unit, input, yj_r);
  }
}


//decalre 45 UGens
void VarMembraneCircle_Ctor(VarMembrane* unit) {
  VarMembrane_init(unit, 0, 0, 0);
}

void VarMembraneHexagon_Ctor(VarMembrane* unit) {
  VarMembrane_init(unit, 1, 0, 0);
}

void StoneChime0_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 2, 2, 0);
}

void StoneChime1_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 2, 6, 0);
}

void StoneChime2_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 2, 10, 0);
}

void StoneChime3_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 2, 14, 0);
}



void SCFrag0_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 0);
}

void SCFrag1_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 1);
}

void SCFrag2_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 2);
}

void SCFrag3_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 3);
}

void SCFrag4_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 4);
}

void SCFrag5_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 5);
}

void SCFrag6_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 6);
}

void SCFrag7_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 7);
}

void SCFrag8_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 8);
}

void SCFrag9_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 9);
}

void SCFrag10_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 10);
}

void SCFrag11_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 11);
}

void SCFrag12_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 12);
}

void SCFrag13_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 13);
}

void SCFrag14_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 14);
}

void SCFrag15_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 15);
}

void SCFrag16_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 16);
}

void SCFrag17_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 17);
}

void SCFrag18_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 18);
}

void SCFrag19_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 19);
}

void SCFrag20_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 20);
}

void SCFrag21_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 21);
}

void SCFrag22_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 22);
}

void SCFrag23_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 23);
}

void SCFrag24_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 24);
}

void SCFrag25_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 25);
}

void SCFrag26_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 26);
}

void SCFrag27_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 27);
}

void SCFrag28_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 28);
}

void SCFrag29_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 29);
}

void SCFrag30_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 30);
}

void SCFrag31_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 31);
}

void SCFrag32_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 32);
}

void SCFrag33_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 33);
}

void SCFrag34_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 34);
}

void SCFrag35_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 35);
}

void SCFrag36_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 36);
}

void SCFrag37_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 37);
}

void SCFrag38_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 38);
}

void SCFrag39_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 39);
}

void SCFrag40_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 40);
}

void SCFrag41_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 41);
}

void SCFrag42_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 42);
}

void SCFrag43_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 43);
}

void SCFrag44_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 44);
}

void SCFrag45_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 45);
}

void SCFrag46_Ctor(VarMembrane* unit){
    VarMembrane_init(unit, 3, 0, 46);
}




void VarMembrane_Dtor(VarMembrane* unit) {
  //메모리 free해준다 
  
  //free_shape(unit->shape);
  RTFree(unit->mWorld, unit->delays);
  RTFree(unit->mWorld, unit->junctions);
}

////////////////////////////////////////////////////////////////////

// the load function is called by the host when the plug-in is loaded
PluginLoad(VarMembrane)
{
  ft = inTable;

  //여기서 2개의 uGen을 만들어 주고 싶은 경우 DefineSimpleUnit을 쓰지 못하는듯. 그건 1개 일때만?
  //아니면 Dtor때문에 그럴수도 
  (*ft->fDefineUnit)("VarMembraneCircle",
		     sizeof(VarMembrane),
		     (UnitCtorFunc)&VarMembraneCircle_Ctor,
		     (UnitDtorFunc)&VarMembrane_Dtor,
		     0);
  (*ft->fDefineUnit)("VarMembraneHexagon",
		     sizeof(VarMembrane),
		     (UnitCtorFunc)&VarMembraneHexagon_Ctor,
		     (UnitDtorFunc)&VarMembrane_Dtor,
		     0);
    
  (*ft->fDefineUnit)("StoneChime0",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&StoneChime0_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);

  (*ft->fDefineUnit)("StoneChime1",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&StoneChime1_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);

  (*ft->fDefineUnit)("StoneChime2",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&StoneChime2_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
  (*ft->fDefineUnit)("StoneChime3",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&StoneChime3_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);


    (*ft->fDefineUnit)("SCFrag0",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag0_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);

    (*ft->fDefineUnit)("SCFrag1",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag1_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag2",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag2_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag3",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag3_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag4",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag4_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag5",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag5_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
      (*ft->fDefineUnit)("SCFrag6",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag6_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag7",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag7_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag8",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag8_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag9",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag9_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag10",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag10_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag11",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag11_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag12",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag12_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag13",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag13_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag14",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag14_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag15",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag15_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag16",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag16_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag17",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag17_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag18",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag18_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag19",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag19_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag20",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag20_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag21",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag21_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag22",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag22_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag23",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag23_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag24",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag24_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag25",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag25_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
      (*ft->fDefineUnit)("SCFrag26",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag26_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag27",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag27_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag28",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag28_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag29",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag29_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
   (*ft->fDefineUnit)("SCFrag30",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag30_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);

    (*ft->fDefineUnit)("SCFrag31",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag31_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag32",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag32_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag33",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag33_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag34",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag34_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag35",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag35_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
      (*ft->fDefineUnit)("SCFrag36",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag36_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag37",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag37_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag38",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag38_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag39",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag39_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);

    (*ft->fDefineUnit)("SCFrag40",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag40_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);

    (*ft->fDefineUnit)("SCFrag41",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag41_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag42",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag42_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag43",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag43_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag44",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag44_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
    (*ft->fDefineUnit)("SCFrag45",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag45_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);
      (*ft->fDefineUnit)("SCFrag46",
                       sizeof(VarMembrane),
                       (UnitCtorFunc)&SCFrag46_Ctor,
                       (UnitDtorFunc)&VarMembrane_Dtor,
                       0);


}

////////////////////////////////////////////////////////////////////

