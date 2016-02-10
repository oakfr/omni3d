#ifndef __COLORS_H__
#define __COLORS_H__

// should probably be called pixels
static float RED[3] = {1.0,0.0,0.0};
static float PINK[3] = {1.0,0.0,1.0};
static float YELLOW[3] = {1.0,1.0,0.0};
static float ORANGE[3] = {1.0,0.5,0.0};
static float GREEN[3] = {0.0,1.0,0.0};
static float BLUEGREEN[3] = {0.0,1.0,1.0};
static float BLUE[3] = {0.0,0.0,1.0};
static float REDGREEN[3]={1.0,1.0,0.0};
static float REDBLUE[3]={1.0,0.0,1.0};
static float DULLRED[3]={0.5,0.0,0.0};
static float DULLBLUE[3]={0.0,0.0,0.5};
static float WHITE[3]={1.0,1.0,1.0};
static float DULLWHITE[3]={0.2,0.2,0.2};
static float GREY[3]={0.2,0.2,0.2};
static float BLACK[3]={0.0,0.0,0.0};
static float REDGREEN2[3]={1.0,0.5,0.0};
static float GRAY[3]={0.2,0.2,0.2};
static float LIGHTBLUE[3]={0.0/255.0,25/255.0,50/255.0};
static float PURPLE[3]={1.0, 0.14, 0.6667};

static int NCOLORS = 11;

const float _colors[11][3] = {
  {1.0, 1.0, 1.0},	// 0: WHITE
  {1.0, 0.0, 0.0},	// 1: RED
  {0.0, 1.0, 0.0},	// 2: GREEN
  {0.0, 0.0, 1.0},	// 3: BLUE
  {1.0, 0.0, 1.0},	// 4: PINK
  {0.0, 1.0, 1.0},	// 5: EMERAUDE
  {1.0, 0.0, 0.0},	// 6: RED
  {0.0, 0.0, 0.0},	// 7: WHITE
  {1.0, 0.5, 0.0},	// 8: ORANGE
  {0.2, 0.2, 0.2},	// 9: GRAY
  {1.0, 0.14, 0.6667},	// 10: PURPLE
};

#endif
