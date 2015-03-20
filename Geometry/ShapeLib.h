#pragma once

//////////////////////////////////////////////////////////////////////////
// Primitive Shapes

// Box
//    7------3
//   /|     /|
//  4-|----0 |
//  | 6----|-2
//  |/     |/
//  5------1
const int psBoxNNbEPerE = 4;

const int boxNumQuadFace = 6;
const int boxQuadFace[6][4] = {
	// quad faces
		{ 0, 1, 2, 3 }, { 6, 5, 4, 7 },
		{ 1, 0, 4, 5 }, { 1, 5, 6, 2 },
		{ 2, 6, 7, 3 }, { 0, 3, 7, 4 }
};

const int boxNumEdge = 12;
const int boxEdge[12][2] = {
		{ 0, 1 }, { 1, 2 },	// 0, 1
		{ 2, 3 }, { 3, 0 },	// 2, 3
		{ 0, 4 }, { 1, 5 },	// 4, 5
		{ 2, 6 }, { 3, 7 },	// 6, 7
		{ 4, 5 }, { 5, 6 },	// 8, 9
		{ 6, 7 }, { 7, 4 }	// 10, 11
};

const int boxNumFace = 12;
const int boxFace[12][3] = {
	// triangular faces
		{ 0, 1, 2 }, { 2, 3, 0 },
		{ 6, 5, 4 }, { 4, 7, 6 },
		{ 1, 0, 4 }, { 4, 5, 1 },
		{ 1, 5, 6 }, { 6, 2, 1 },
		{ 2, 6, 7 }, { 7, 3, 2 },
		{ 0, 3, 7 }, { 7, 4, 0 }
};

const int boxFaceNormalOrientAlongAxis[12][2] = {
	// face normal orientation along axis
	// {axis_id (along which axis), orientation (1 means pointing to positive and -1 to negative)}
		{ 0, 1 }, { 0, 1 },
		{ 0, -1 }, { 0, -1 },
		{ 2, 1 }, { 2, 1 },
		{ 1, -1 }, { 1, -1 },
		{ 2, -1 }, { 2, -1 },
		{ 1, 1 }, { 1, 1 }
};