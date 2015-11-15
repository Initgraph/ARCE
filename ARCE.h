//
// ARCE : Arduboy Ray Casting Engine
//
// Copyright (C) 2015 Jerome Perrot (Initgraph)
//
// Version : 0.1
//
// Notes :
//
//   You need the Arduboy library in order to compile this program, please see the link below for more explanations :
//   
//   http://community.arduboy.com/t/getting-started-with-the-arduboy
//
//   Each block in a map loaded in the engine are 1 unit wide (1 x 1). In some raycasting engines, all the moves and rotations (player, ennemies, rays, 
//   etc...) are calculated inside the map area and floating-point calculations are used. 
//   The Arduboy processor can use floats but using them in a raycasting engine is really too slow :-( 
//   In order to use only integers, this engine does not perfoms moves and rotations calculations inside the map area (called "worldMap" in this
//   engine) but inside another area called "world". Each block in the world are 64 unit wide (64 x 64) by default (this value is defined with the 
//   BLOCK_SIZE constant).
//   The world coordinates are then converted into screen coordinates during the projection step of the rendering process.
//   All rotations are performed inside a screen oriented cartesian coordinate system : the y axis is inverted and the first quadrant is on the bottom
//   right of the coordinate system. 
//   All angle values are expressed in degrees.
// 
// Special thanks :
//
//   - "Programmation graphique en C" book by Walter Siracusa (ISBN : 2-7361-2136-8) : this book has taught me some 3D techniques and how to use integers 
//     instead of floats.
//   - "Tricks of the Game-Programming Gurus" book by Andre Lamothe and John Ratcliff (ISBN-10: 0672305070) : this book has taught me how raycasting works 
//     and some other tricks.
//   - Permadi website (http://permadi.com/1996/05/ray-casting-tutorial-table-of-contents) : a very good raycasting tutorial.
//   - Jacob Seidelin website (https://dev.opera.com/articles/3d-games-with-canvas-and-raycasting-part-1) : another raycasting tutorial with good HTML 
//     examples.
//
// Licence :  
//
//   This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by
//   the Free Software Foundation; either version 2 of the License, or (at your option) any later version.
//   This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
//   You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc.,
//   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
// For any remarks, suggestions or questions you can contact me with this email address : contact@initgraph.com
// You can also visit my website : http://www.initgraph.com 
//
// Enjoy !
//

#ifndef ARCE_H
#define ARCE_H

#include <SPI.h>
#include <EEPROM.h>
#include "Arduboy.h"

// Arduboy device constants
#define SCREEN_WIDTH 128      // Arduboy screen width.     
#define SCREEN_HEIGHT 64      // Arduboy screen height. 
#define HALF_SCREEN_WIDTH 64  // Half Arduboy screen width. 
#define HALF_SCREEN_HEIGHT 32 // Half Arduboy screen height.
#define KEY_UP 8              // Constant for the "UP" button. Can be used with the Arduino digitalRead function.
#define KEY_DOWN 10           // Constant for the "DOWN" button. Can be used with the Arduino digitalRead function.
#define KEY_LEFT 9            // Constant for the "LEFT" button. Can be used with the Arduino digitalRead function.
#define KEY_RIGHT 5           // Constant for the "RIGHT" button. Can be used with the Arduino digitalRead function.
#define KEY_A A0              // Constant for the "A" button. Can be used with the Arduino digitalRead function.
#define KEY_B A1              // Constant for the "B" button. Can be used with the Arduino digitalRead function.

// ARCE constants
#define PLAYER_MOVE_NONE 0             // Can be used with the ARCEPlayer.moveDir variable.
#define PLAYER_MOVE_FORWARD 1          // Can be used with the ARCEPlayer.moveDir variable.
#define PLAYER_MOVE_BACKWARD -1        // Can be used with the ARCEPlayer.moveDir variable.
#define PLAYER_ROTATE_NONE 0           // Can be used with the ARCEPlayer.rotDir variable.
#define PLAYER_ROTATE_LEFT -1          // Can be used with the ARCEPlayer.rotDir variable.
#define PLAYER_ROTATE_RIGHT 1          // Can be used with the ARCEPlayer.rotDir variable.
#define VIEW_2D_ONERAY 0               // 2D view with player orientation ray. Can be used with the ARCE.view variable.   
#define VIEW_2D 1                      // 2D view with all rays. Can be used with the ARCE.view variable. 
#define VIEW_3D_SOLID 2                // 3D view without textures. Can be used with the ARCE.view variable. 
#define VIEW_3D_TEXTURED 3             // 3D view with textures. Can be used with the ARCE.view variable. 
#define TEXTURE_ORIENT_LEFT_TO_RIGHT 0 // Texture orientation. The texture have to be render from left to right. 
#define TEXTURE_ORIENT_RIGHT_TO_LEFT 1 // Texture orientation. The texture have to be render from right to left.
#define MULTIPLY_BY_2 1                // Can be used in a bit shift operation in order to multiply a value by 2. 
#define DIVIDE_BY_2 1                  // Can be used in a bit shift operation in order to divide a value by 2.
#define MULTIPLY_BY_8 3                // Can be used in a bit shift operation in order to multiply a value by 8.
#define DIVIDE_BY_8 3                  // Can be used in a bit shift operation in order to divide a value by 8.
#define MULTIPLY_BY_128 7              // Can be used in a bit shift operation in order to multiply a value by 128.
#define DIVIDE_BY_128 7                // Can be used in a bit shift operation in order to divide a value by 128.

// ARCE settings
#define FOV 64                               // Field of view of the player. 60 degrees is a good choice for a nice view but 64 is a better value because it's the half width of the screen.
#define HALF_FOV 32                          // Half field of view of the player.
#define BLOCK_SIZE 64                        // Block size in the world (world coordinates).
#define MULTIPLY_BY_BLOCK_SIZE 6             // Can be used in a bit shift operation in order to multiply a value by the block size.
#define DIVIDE_BY_BLOCK_SIZE 6               // Can be used in a bit shift operation in order to divide a value by the block size.
#define PROJECTION_K 6528                    // Constant used for projection. You can find an explanation of this constant at the projection section in the ARCE.cpp source code.
#define K 128                                // Constant used to perform floating point calculations with integers.
#define MULTIPLY_BY_K 7                      // Can be used in a bit shift operation in order to multiply a value by K.
#define DIVIDE_BY_K 7                        // Can be used in a bit shift operation in order to divide a value by K.
#define TEXTURE_SIZE 32                      // Texture size. 32 value was chosen because it's the half size of a block.
#define TEXTURE_SIZE_BY_K 4096               // Texture size mutiplied by K.
#define TEXTURE_SCALING_FACTOR 2             // Texture scaling factor (tell how to apply a 32x32 texture on a 64x64 block).
#define MULTIPLY_BY_TEXTURE_SCALING_FACTOR 1 // Can be used in a bit shift operation in order to multiply a value by the texture scaling factor.
#define DIVIDE_BY_TEXTURE_SCALING_FACTOR 1   // Can be used in a bit shift operation in order to divide a value by the texture scaling factor.
#define PLAYER_COLLISION_MIN_DIST 1          // Constant used to calculate the minimal distance between the player and a block.  
#define WORLD_TO_SCREEN_SCALING_FACTOR 16    // Scaling factor between world coordinates and screen coordinates.

// Cosinus array for player rotation.
// Each cosinus value is multiplied by 16 in order to use integers instead of floats.
PROGMEM const int8_t cosBy16[360] = {
  
  16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 
  16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 
  15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 
  14, 14, 14, 13, 13, 13, 13, 13, 13, 12, 
  12, 12, 12, 12, 12, 11, 11, 11, 11, 10, 
  10, 10, 10, 10, 9, 9, 9, 9, 8, 8, 
  8, 8, 8, 7, 7, 7, 7, 6, 6, 6, 
  5, 5, 5, 5, 4, 4, 4, 4, 3, 3, 
  3, 3, 2, 2, 2, 1, 1, 1, 1, 0, 
  0, 0, -1, -1, -1, -1, -2, -2, -2, -3, 
  -3, -3, -3, -4, -4, -4, -4, -5, -5, -5, 
  -5, -6, -6, -6, -7, -7, -7, -7, -8, -8, 
  -8, -8, -8, -9, -9, -9, -9, -10, -10, -10,
  -10, -10, -11, -11, -11, -11, -12, -12, -12, -12, 
  -12, -12, -13, -13, -13, -13, -13, -13, -14, -14, 
  -14, -14, -14, -14, -14, -15, -15, -15, -15, -15, 
  -15, -15, -15, -15, -15, -15, -16, -16, -16, -16, 
  -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, 
  -16, -16, -16, -16, -16, -16, -16, -16, -16, -16, 
  -16, -16, -16, -16, -16, -15, -15, -15, -15, -15, 
  -15, -15, -15, -15, -15, -15, -14, -14, -14, -14, 
  -14, -14, -14, -13, -13, -13, -13, -13, -13, -12, 
  -12, -12, -12, -12, -12, -11, -11, -11, -11, -10, 
  -10, -10, -10, -10, -9, -9, -9, -9, -8, -8, 
  -8, -8, -8, -7, -7, -7, -7, -6, -6, -6, 
  -5, -5, -5, -5, -4, -4, -4, -4, -3, -3, 
  -3, -3, -2, -2, -2, -1, -1, -1, -1, 0, 
  0, 0, 1, 1, 1, 1, 2, 2, 2, 3, 
  3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 
  5, 6, 6, 6, 7, 7, 7, 7, 8, 8, 
  8, 8, 8, 9, 9, 9, 9, 10, 10, 10,
  10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 
  12, 12, 13, 13, 13, 13, 13, 13, 14, 14, 
  14, 14, 14, 14, 14, 15, 15, 15, 15, 15,
  15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 
  16, 16, 16, 16, 16, 16, 16, 16, 16, 16,
};

// Cosinus array for rays rotations.
// Each cosinus value is multiplied by 128 in order to use integers instead of floats.
PROGMEM const uint8_t cosBy128[91] = {
  
  128, 128, 128, 128, 128, 128, 127, 127, 127, 126, 
  126, 126, 125, 125, 124, 124, 123, 122, 122, 121, 
  120, 119, 119, 118, 117, 116, 115, 114, 113, 112, 
  111, 110, 109, 107, 106, 105, 104, 102, 101, 99, 
  98, 97, 95, 94, 92, 91, 89, 87, 86, 84, 
  82, 81, 79, 77, 75, 73, 72, 70, 68, 66, 
  64, 62, 60, 58, 56, 54, 52, 50, 48, 46, 
  44, 42, 40, 37, 35, 33, 31, 29, 27, 24, 
  22, 20, 18, 16, 13, 11, 9, 7, 4, 2, 0
};

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Player Class
// ------------------------------------------------------------------------------------------------------------------------------------------------------
class ARCEPlayer {
  
  public:
  
    int16_t x = 96;                    // X position of the player in the world (world coordinates).
    int16_t y = 96;                    // Y position of the player in the world (world coordinates).
    int8_t moveDir = PLAYER_MOVE_NONE; // Direction of the player move : PLAYER_MOVE_NONE, PLAYER_MOVE_FORWARD or PLAYER_MOVE_BACKWARD.
    uint8_t moveStep = 1;              // Step value of the player move (speed).
    int16_t rot = 0;                   // Angle of the player rotation (degrees).
    int8_t rotDir = 0;                 // Direction of the player rotation : PLAYER_ROTATE_NONE, PLAYER_ROTATE_LEFT or PLAYER_ROTATE_RIGHT.
    uint8_t rotStep = 5;               // Step value angle of the player rotation (rotation speed, expressed in degrees).

    ARCEPlayer();                      // Player Class constructor.
};

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// ARCE Engine Class
// ------------------------------------------------------------------------------------------------------------------------------------------------------
class ARCE {
  
  public:
    
    ARCEPlayer player;                 // Player object.
    Arduboy display;                   // Arduboy library object.
    uint8_t view = VIEW_3D_TEXTURED;   // Current view : VIEW_2D_ONERAY, VIEW_2D, VIEW_3D_SOLID or VIEW_3D_TEXTURED.
    const uint8_t *texturesArray[256]; // Textures array : texturesArray[0] is used with block "1" in world map, etc...
    
    ARCE();                                            // ARCE Engine Class constructor    
    void start();                                      // Initialize ARCE Engine. Can be called inside the Arduino "setup()" function. Used instead of "ARCE()" constructor.
    void update();                                     // Must be called every frame. Can be placed inside the Arduino "loop()" function.
    void castRay(uint8_t rayNumber, int16_t rayAngle); // Cast a ray with a given number and a given angle.
    void loadWorldMap(const uint8_t *worldMap, uint8_t worldMapWidth, uint8_t worldMapHeight);                              // Load a given world map in the engine.
    uint8_t getTexel (uint8_t texelX, uint8_t texelY, const uint8_t *texture, uint8_t textureWidth, uint8_t textureHeight); // Read a pixel from a given texture.
    
  private:
    
    const uint8_t *worldMap;    // Current world map. 
    uint8_t worldMapWidth = 0;  // World map width.
    uint8_t worldMapHeight = 0; // World map height.
    uint16_t worldWidth = 0;    // World width.
    uint16_t worldHeight = 0;   // World height.
};

#endif
