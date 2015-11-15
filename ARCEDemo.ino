//
// ARCE demo : Arduboy Ray Casting Engine demo
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

#include <SPI.h>
#include <EEPROM.h>
#include <stdio.h>
#include <Arduboy.h>
#include "ARCE.h"

// Create a wall texture
PROGMEM const uint8_t wall1[] = {
  
  B01111111,B11111101,B11111101,B11110111,
  B01111111,B11111101,B11111101,B11110111,
  B01111111,B11111101,B11111101,B11110111,
  B01111111,B11111101,B11111101,B11110000,
  B01111111,B11111101,B11111101,B11110111,
  B01111111,B11111101,B11111101,B11110111,
  B01111111,B11111101,B11111101,B11110111,
  B00000000,B00000000,B00000001,B11110000,
  B11111110,B11111111,B10111101,B11110111,
  B11111110,B11111111,B10111101,B11110111,
  B11111110,B11111111,B10111101,B11110111,
  B11111110,B11111111,B10000001,B11110111,
  B11111110,B11111111,B10111101,B11110111,
  B11111110,B11111111,B10111101,B11110111,
  B11111110,B11111111,B10111101,B11110111,
  B11111110,B11111111,B10111101,B11110111,
  B11111110,B11111111,B10111101,B11110111,
  B11111110,B00000000,B00111101,B11110111,
  B11111110,B11111111,B10111101,B11110111,
  B11111110,B11111111,B10111101,B11110111,
  B11111110,B11111111,B10111101,B11110111,
  B00000000,B00000000,B00000000,B00000000,
  B01111111,B11110111,B01111101,B11111111,
  B01111111,B11110111,B01111101,B11111111,
  B01111111,B11110111,B01111101,B11111111,
  B01111111,B11110000,B00000001,B11111111,
  B01111111,B11110111,B11111101,B11111111,
  B01111111,B11110111,B11111101,B11111111,
  B01111111,B11110111,B11111101,B11111111,
  B01111111,B11110111,B11111101,B11111111,
  B01111111,B11110111,B11111101,B11111111,
  B01111111,B11110111,B11111101,B11111111
};

// Create a second wall texture
PROGMEM const uint8_t wall2[] = {
  
  B01111111,B11111101,B11111101,B11110111,
  B01111111,B11111101,B11111101,B11110111,
  B01111111,B11111101,B11111101,B11110111,
  B01111111,B11111101,B11111101,B11110000,
  B01111111,B11111101,B11111101,B11110111,
  B01111000,B00000000,B00000000,B00010111,
  B01111011,B11111111,B11111111,B10010111,
  B00000010,B01001001,B00100100,B10010000,
  B11111010,B01001001,B00100100,B10010111,
  B11111010,B01001001,B00100100,B11010111,
  B11111010,B01001001,B00100100,B11010111,
  B11111010,B01001001,B00100100,B11010111,
  B11111010,B01001001,B00100100,B11010111,
  B11111010,B01001001,B00100100,B10010111,
  B11111011,B11101001,B00100100,B10010111,
  B11111011,B11101001,B00100100,B10010111,
  B11111011,B11101001,B00100100,B10010111,
  B11111011,B11101001,B00100100,B10010111,
  B11111010,B01101001,B00100100,B10010111,
  B11111010,B01101001,B00100100,B10010111,
  B11111011,B11101001,B00100100,B10010111,
  B00000011,B11101001,B00100100,B10000000,
  B01111011,B11101001,B00100100,B10011111,
  B01111011,B11101001,B00100100,B10011111,
  B01111010,B01001001,B00100100,B10011111,
  B01111010,B01001001,B00100100,B11011111,
  B01111010,B01001001,B00100100,B11011111,
  B01111010,B01001001,B00100100,B11011111,
  B01111010,B01001001,B00100100,B11011111,
  B01111010,B01001001,B00100100,B10011111,
  B01111010,B01001001,B00100100,B10011111,
  B01111011,B11111111,B11111111,B10011111
};

// Create a door texture
PROGMEM const uint8_t door[] = {
  
  B00000000,B00000000,B00000000,B00000000,
  B01111111,B11111111,B11111111,B11111110,
  B01011011,B01101101,B10110110,B11011010,
  B01111111,B11111111,B11111111,B11111110,
  B01110000,B00000000,B00000000,B00001110,
  B01010111,B11111111,B11111111,B11101010,
  B01110111,B11111111,B11111111,B11101110,
  B01110111,B11111111,B11111111,B11101110,
  B01010111,B11111111,B11111111,B11101010,
  B01110111,B11111111,B11111111,B11101110,
  B01110111,B11111111,B11111111,B11101110,
  B01010111,B11111111,B11111111,B11101010,
  B01110111,B00000011,B11111111,B11101110,
  B01110101,B01111011,B11111111,B11101110,
  B01010101,B01111011,B11111111,B11101010,
  B01110101,B01001011,B11111111,B11101110,
  B01110101,B01001011,B11111111,B11101110,
  B01010101,B01111011,B11111111,B11101010,
  B01110101,B01111011,B11111111,B11101110,
  B01110111,B00000011,B11111111,B11101110,
  B01010111,B11111111,B11111111,B11101010,
  B01110111,B11111111,B11111111,B11101110,
  B01110111,B11111111,B11111111,B11101110,
  B01010111,B11111111,B11111111,B11101010,
  B01110111,B11111111,B11111111,B11101110,
  B01110111,B11111111,B11111111,B11101110,
  B01010111,B11111111,B11111111,B11101010,
  B01110000,B00000000,B00000000,B00001110,
  B01111111,B11111111,B11111111,B11111110,
  B01011011,B01101101,B10110110,B11011010,
  B01111111,B11111111,B11111111,B11111110,
  B00000000,B00000000,B00000000,B00000000
};

// Create a 32 x 16 demo map
PROGMEM const uint8_t demoMap[512] = {
  
  1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  2,0,0,0,1,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  1,0,0,0,1,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  1,0,0,0,1,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  2,0,0,0,1,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  1,0,0,0,1,1,0,1,1,1,1,0,1,1,1,1,1,1,1,3,1,1,1,1,1,1,0,0,0,0,0,0,
  1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,
  2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,3,0,0,0,0,0,0,
  1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,
  1,0,0,0,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,3,1,1,1,1,1,1,0,0,0,0,0,0,
  2,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  1,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  2,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};

// Create the ARCE object
ARCE arce;

// Strings for displaying current key and current view
char view[15];
char key[10];

// FPS calculation variables
uint32_t time = 0;
uint32_t previousTime = 0;
uint8_t fps = 0;

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Arduino setup function
// ------------------------------------------------------------------------------------------------------------------------------------------------------
void setup() {
  
  // Initialize ARCE
  arce.start();
  
  // Load the 32 x 16 demo map
  arce.loadWorldMap(demoMap, 32, 16);
  
  // Add textures
  arce.texturesArray[0] = wall1; // texturesArray[0] is used with block "1" in world map
  arce.texturesArray[1] = wall2; // texturesArray[1] is used with block "2" in world map
  arce.texturesArray[2] = door;  // texturesArray[2] is used with block "3" in world map
  
  // Initialize player position and rotation
  arce.player.x = 416;
  arce.player.y = 192;
  arce.player.rot = 90;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Arduino loop function
// ------------------------------------------------------------------------------------------------------------------------------------------------------
void loop() {
  
  // Clear strings
  sprintf(view, "");
  sprintf(key, "");
  
  // Clear display
  arce.display.clearDisplay();
  
  // Read keys
  if(!digitalRead(KEY_UP)) {
    
    sprintf(key, "UP");
    arce.player.moveDir = PLAYER_MOVE_FORWARD;
  }
  
  if(!digitalRead(KEY_DOWN)) {
    
    sprintf(key, "DOWN");
    arce.player.moveDir = PLAYER_MOVE_BACKWARD;
  }
  
  if(!digitalRead(KEY_LEFT)) {
    
    sprintf(key, "LEFT");
  }
  
  if(!digitalRead(KEY_RIGHT)) {
    
    sprintf(key, "RIGHT");
    
    // Update ARCE view
    arce.view += 1;
    arce.view %= 4;
    
    // Display ARCE view
    if (arce.view == VIEW_2D_ONERAY) {
      
      sprintf(view, "2D_ONERAY");
    }
    else if (arce.view == VIEW_2D) {
      
      sprintf(view, "2D");
    }
    else if (arce.view == VIEW_3D_SOLID) {
      
      sprintf(view, "3D_SOLID");
    }
    else {
      
      sprintf(view, "3D_TEXTURED");
    }
    
    // Add a delay in order to switch between views easily
    delay(150);
  }
  
  if(!digitalRead(KEY_A)) {
    
    sprintf(key, "A");
    arce.player.rotDir = PLAYER_ROTATE_LEFT;
  }
  
  if(!digitalRead(KEY_B)) {
     
    sprintf(key, "B");
    arce.player.rotDir = PLAYER_ROTATE_RIGHT;
  }
    
  // Update ARCE (Player movement and rotation, etc...)
  arce.update();
  
  // Show FPS (for debug only) 
  time = millis();
  fps = 1000.0f / (time - previousTime);
  arce.display.setCursor(100, 0);
  arce.display.print(fps); 
  previousTime = time;
  
  // Show current player rotation angle
  arce.display.setCursor(100, 10);
  arce.display.print(arce.player.rot);
  
  // Show current key
  arce.display.setCursor(100, 20);
  arce.display.print(key);
  
  // Show current view  
  arce.display.setCursor(0, 0);
  arce.display.print(view);

  // Update Display
  arce.display.display();
}
