/* **************************************************
          SHAPES
************************************************** */

#include "Shapes.hpp"
Shape gLine {
  .size = 4,
  .data = { {50, 0}, {0, -50}, {-50, 0}, {0, 50} }
};

Shape gStar {
  .size = 83,
  .data = {
    {0,0},      {-71, -71}, {-71, -71}, {-69, -73}, {-99, 5},
    {-100, 5},  {-100, 10}, {-99, 15},  {-98, 20},  {-97, 25},
    {-96, 29},  {-95, 31},  {-50, -88}, {-50, -88}, {-45, -90},
    {-41, -92}, {-36, -94}, {-33, -95}, {-88, 48},  {-88, 48},
    {-85, 52},  {-83,56},   {-80, 60},  {-80, 61},  {-18, -99},
    {-18, -99}, {-13,-100}, {-8, -100}, {-3, -100}, {-70, 71},
    {-70, 71},  {-67, 75},  {-63, 78},  {-60, 80},  {11, -100},
    {11, -100}, {16, -99},  {20, -98},  {24, -98},  {-48, 87},
    {-48, 87},  {-44, 90},  {-39, 92},  {-37, 93},  {36, -94},
    {36, -94},  {41, -92},  {45, -90},  {48, -88},  {-24, 97},
    {-24, 97},  {-19, 98},  {-14, 99},  {-11, 99},  {59, -80},
    {59, -81},  {63, -78},  {67, -75},  {69, -72},  {3, 99},
    {3, 99},    {8, 99},    {12, 99},   {17, 98},   {79, -61},
    {79, -61},  {82, -57},  {85, -53},  {87, -49},  {33, 93},
    {33, 94},   {38, 92},   {42, 90},   {46, 88},   {50,86},
    {94, -32},  {94, -32},  {96, -27},  {97, -22},  {98,-17},
    {99, -13},  {99, -8},   {69, 71}
  }
};

Shape gStar2 {
  .size = 55,
  .data = {
    {-11, 5},   {-21, -9},  {-4, -9},   {5, -23},   {10, -6},
    {25, -1},   {12, 9},    {12, 26},   {-1,16},    {-16, 21},
    {-11, 5},   {-23, 7},   {-42, -19}, {-11, -18}, {6, -44},
    {15, -14},  {44, -4},   {20, 14},   {20, 46},   {-4 ,27},
    {-33, 37},  {-23, 7},   {-33, 8},   {-58, -27}, {-17, -26},
    {7, -61},   {19, -20},  {59, -6},   {25, 19},   {26, 62},
    {-7, 36},   {-46, 49},  {-33, 8},   {-41, 9},   {-71, -33},
    {-21, -32}, {8, -74},   {23, -24},  {70, -8},   {30, 22},
    {31, 74},   {-9, 43},   {-56, 59},  {-41 ,9},   {-48, 11},
    {-83, -39}, {-25, -37}, {8, -87},   {26, -29},  {82, -10},
    {34, 25},   {36, 86},   {-11, 49},  {-66, 69},  {-48, 11}
  }
};

Shape gSpiral = {
  .size = 60,
  .data = {
    {3,0},      {-7, 4},    {-14, -5},  {-6, -16},  {8, -11}, 
    {7, 6},     {-10, 10},  {-19, -7},  {-5, -22},  {15, -11}, 
    {10, 11},   {-14, 13},  {-23, -11}, {-2, -26},  {20, -10}, 
    {10, 17},   {-19, 15},  {-25, -14}, {2, -29},   {25, -7}, 
    {9, 22},    {-24, 15},  {-27, -19}, {7, -32},   {29, -2}, 
    {7, 29},    {-30, 16},  {-28, -25}, {13, -35},  {35, 3}, 
    {4, 37},    {-38, 15},  {-30, -33}, {21, -38},  {42, 11}, 
    {-1, 46},   {-48, 14},  {-30, -43}, {32, -41},  {49, 22}, 
    {-9, 57},   {-60, 10},  {-29, -54}, {45, -42},  {54, 35}, 
    {-19, 68},  {-73, 4},   {-25, -67}, {59, -41},  {58, 51}, 
    {-33, 77},  {-84, -5},  {-19, -79}, {74, -36},  {58, 68}, 
    {-48, 83},  {-94, -17}, {-10, -90}, {87, -28},  {54, 83}
  }
};

Shape gClearLines = {
  .size = 160,
  .data = {
    {59, 79}, {7, 99}, {3, 99}, {-2, 99}, {-7, 99},
    {-7, 99}, {71, 69}, {72, 69}, {75, 65}, {78, 61},
    {78, 61}, {-18, 98}, {-22, 97}, {-27, 96}, {-27, 96}, 
    {83, 54}, {85, 51}, {88, 47}, {88, 47}, {-34, 94},
    {-36, 93}, {-40, 91}, {-40, 90}, {91, 40}, {92, 39},
    {93, 34}, {93, 34}, {-47, 88}, {-49, 87}, {-53, 85},
    {-53, 85}, {95, 28}, {96, 27}, {97, 22}, {97, 22},
    {-58, 81}, {-59, 81}, {-63, 78}, {-62, 78}, {98, 16},
    {98, 15}, {99, 10}, {99, 10}, {-67, 74}, {-68, 74},
    {-71, 70}, {-71, 70}, {99, 5}, {99, 4}, {100, -1},
    {100, -1}, {-75, 66}, {-75, 66}, {-78, 62}, {-78, 62},
    {99, -6}, {99, -6}, {99, -11}, {99, -11}, {-81, 58},
    {-82, 58}, {-84, 54}, {-84, 54}, {98, -16}, {98, -16},
    {97, -21}, {97, -21}, {-87, 50}, {-87, 50}, {-89, 45},
    {-89, 45}, {96, -26}, {96, -26}, {95, -31}, {95, -31},
    {-92, 41}, {-92, 41}, {-94, 36}, {-94, 36}, {93, -36},
    {93, -36}, {91, -40}, {91, -40}, {-95, 31}, {-95, 31},
    {-97, 27}, {-97, 27}, {89, -45}, {89, -45}, {87, -49},
    {87, -49}, {-98, 22}, {-98, 22}, {-99, 17}, {-99, 17},
    {84, -54}, {84, -54}, {81, -58}, {81, -58}, {-100, 12},
    {-100, 11}, {-100, 7}, {-100, 7}, {78, -62}, {78, -62},
    {75, -66}, {75, -66}, {-100, 1}, {-100, 1}, {-100, -4},
    {-100, -4}, {71, -70}, {71, -70}, {67, -74}, {67, -74},
    {-100, -10}, {-100, -10}, {-99, -15}, {-99, -15}, {63, -78},
    {62, -78}, {59, -81}, {58, -81}, {-98, -21}, {-98, -23},
    {-97, -28}, {-96, -27}, {54, -85}, {52, -85}, {48, -88},
    {48, -88}, {-95, -33}, {-94, -35}, {-93, -39}, {-92, -39},
    {42, -91}, {40, -92}, {36, -94}, {36, -94}, {-90, -46},
    {-88, -48}, {-86, -53}, {-86, -53}, {28, -96}, {24, -97},
    {20, -98}, {20, -98}, {-81, -60}, {-81, -60}, {-78, -64},
    {-75, -68}, {-75, -68}, {10, -100}, {7, -100}, {2, -100},
    {-3, -100}, {-3, -100}, {-66, -76}, {-68, -75}, {-71, -71}
  }
};
