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

#include "ARCE.h"

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// ARCE Player Class constructor.
// ------------------------------------------------------------------------------------------------------------------------------------------------------
ARCEPlayer::ARCEPlayer() { }

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// ARCE Engine Class constructor.
// /!\ Calling Arduboy::start() function in this constructor breaks the device. ARCE::start() is used instead of this constructor /!\
// ------------------------------------------------------------------------------------------------------------------------------------------------------
ARCE::ARCE() { }

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Initialize ARCE Engine. Can be placed inside the Arduino "setup()" function. Used instead of "ARCE()" constructor. 
// ------------------------------------------------------------------------------------------------------------------------------------------------------
void ARCE::start() {
  
  SPI.begin();
  
  // Initialize Arduboy library
  display.start();
  display.setTextSize(1);
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Must be called every frame. Can be placed inside the Arduino "loop()" function.
// ------------------------------------------------------------------------------------------------------------------------------------------------------
void ARCE::update() {
  
  int16_t playerRotForSin = 0;             // Player rotation angle for calculate player rotation angle sinus. Sinus value can be calculated from a cosinus value : Sin(A) = Cos(A - 90).                                       
  int8_t playerRotCosBy16 = 0;             // Player rotation angle cosinus. This value is multiplied by 16 in order to use integers.
  int8_t playerRotSinBy16 = 0;             // Player rotation angle sinus. This value is multiplied by 16 in order to use integers.
  
  int8_t playerMoveForColCheck = 0;        // Player move for collision check. This value is not used to calculate the final player move,
                                           // it is only usefull for maintain a minimal distance between the player and a obstacle.
                                           
  uint16_t nextPlayerXForColCheck = 0;     // Next player X position for collision check. This value is not used to calculate the final player position, 
                                           // it is only usefull for maintain a minimal distance between the player and a obstacle (word coordinates).
                                           
  uint16_t nextPlayerYForColCheck = 0;     // Next player Y position for collision check. This value is not used to calculate the final player position, 
                                           // it is only usefull for maintain a minimal distance between the player and a obstacle (word coordinates).
                                           
  uint8_t nextPlayerXOnMapForColCheck = 0; // Next player X position on the world map for collision check. This value is not used to calculate the final player position,
                                           // it is only usefull for maintain a minimal distance between the player and a obstacle (word map coordinates). 
                                           
  uint8_t nextPlayerYOnMapForColCheck = 0; // Next player Y position on the world map for collision check. This value is not used to calculate the final player position,
                                           // it is only usefull for maintain a minimal distance between the player and a obstacle (word map coordinates). 
                                           
  int8_t playerMove = 0;                                                   // Player move.
  uint16_t newPlayerX = 0;                                                 // New player X position (word coordinates).
  uint16_t newPlayerY = 0;                                                 // New player Y position (word coordinates).
  uint8_t newPlayerXOnMap = 0;                                             // New player X position on the world map (word map coordinates).
  uint8_t newPlayerYOnMap = 0;                                             // New player Y position on the world map (word map coordinates).
  uint8_t blockSizeOnScreen = BLOCK_SIZE / WORLD_TO_SCREEN_SCALING_FACTOR; // Block size on the screen (screen coordinates). This value is usefull for 2D view.
  uint8_t playerXOnScreen = player.x / WORLD_TO_SCREEN_SCALING_FACTOR;     // New player X position on the screen (screen coordinates). This value is usefull for 2D view.
  uint8_t playerYOnScreen = player.y / WORLD_TO_SCREEN_SCALING_FACTOR;     // New player Y position on the screen (screen coordinates). This value is usefull for 2D view.
  int16_t rayAngle;                                                        // Ray Angle used for cast a ray.

  // Update player rotation. Rotation angle should remain between 0 and 360 degrees.
  player.rot += player.rotDir * player.rotStep;
  player.rot %= 360;
  if (player.rot < 0) player.rot += 360;
  playerRotForSin = player.rot - 90;
  if (playerRotForSin < 0) playerRotForSin += 360;
  playerRotCosBy16 = pgm_read_byte(cosBy16 + player.rot);
  playerRotSinBy16 = pgm_read_byte(cosBy16 + playerRotForSin);

  // Prepare player collision check
  playerMoveForColCheck = player.moveDir * (player.moveStep + PLAYER_COLLISION_MIN_DIST);
  nextPlayerXForColCheck = player.x + playerRotCosBy16 * playerMoveForColCheck;
  nextPlayerYForColCheck = player.y + playerRotSinBy16 * playerMoveForColCheck;
  nextPlayerXOnMapForColCheck = nextPlayerXForColCheck >> DIVIDE_BY_BLOCK_SIZE;
  nextPlayerYOnMapForColCheck = nextPlayerYForColCheck >> DIVIDE_BY_BLOCK_SIZE;
  
  // If the next player position is inside the world and outside a obstacle
  if (nextPlayerXForColCheck >= 0 && nextPlayerXForColCheck < worldWidth && 
      nextPlayerYForColCheck >= 0 && nextPlayerYForColCheck < worldHeight && 
      pgm_read_byte(worldMap + nextPlayerYOnMapForColCheck * worldMapWidth + nextPlayerXOnMapForColCheck) == 0) {
        
      // The next player position is OK and updated
      playerMove = player.moveDir * player.moveStep;
      newPlayerX = player.x + playerRotCosBy16 * playerMove;
      newPlayerY = player.y + playerRotSinBy16 * playerMove;
      newPlayerXOnMap = newPlayerX >> DIVIDE_BY_BLOCK_SIZE;
      newPlayerYOnMap = newPlayerY >> DIVIDE_BY_BLOCK_SIZE;  
      player.x = newPlayerX;
      player.y = newPlayerY;
  }
    
  // If the view is a 2D view, draw the world map with the player on the screen
  if (view == VIEW_2D_ONERAY || view == VIEW_2D) {
    
    // Draw the world map    
    for (uint8_t blockY=0; blockY<worldMapHeight; blockY++) {
      
      for (uint8_t blockX=0; blockX<worldMapWidth; blockX++) {
       
        if (pgm_read_byte(worldMap + blockY * worldMapWidth + blockX) > 0) {
  
  	  display.drawRect(blockX * blockSizeOnScreen, blockY * blockSizeOnScreen, blockSizeOnScreen, blockSizeOnScreen, 1);
        }
      }
    }
    
    // Draw the player 
    display.drawRect(playerXOnScreen - 1, playerYOnScreen - 1, 2, 2, 1);
  }
  
  // If the view is the VIEW_2D_ONERAY view
  if (view == VIEW_2D_ONERAY) {
    
    // Cast player rotation ray
    castRay(32, player.rot);
  }
  
  // If the view is the VIEW_2D view
  else {
  
    // Cast player field of view rays
    rayAngle = player.rot - 32;
    for (uint8_t rayNumber=0; rayNumber<64; rayNumber++) {
      
      castRay(rayNumber, rayAngle);
      rayAngle++;
    }
  }
  
  // Reset player move and rotation for next frame
  player.moveDir = PLAYER_MOVE_NONE;
  player.rotDir = PLAYER_ROTATE_NONE;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Cast a ray with a given number and a given angle.
// ------------------------------------------------------------------------------------------------------------------------------------------------------
void ARCE::castRay(uint8_t rayNumber, int16_t rayAngle) {
  
  uint8_t vccCosBy128 = 0;                // Ray angle cosinus multiplied by 128 for vertical collision check (vcc).
  uint16_t vccSinBy128 = 0;               // Ray angle sinus multiplied by 128 for vertical collision check.
  uint16_t vccTanByBlockSize = 0;         // Ray angle tangente multiplied by BLOCK_SIZE for vertical collision check.
  int16_t vccX = 0;                       // X position of the vertical collision check (world coordinates).
  int16_t vccY = 0;                       // Y position of the vertical collision check (world coordinates).
  int16_t vccStepX = 0;                   // Step to make along the X axis during the vertical collision check (world coordinates).
  int16_t vccStepY = 0;                   // Step to make along the Y axis during the vertical collision check (world coordinates).
  uint8_t hccCosBy128 = 0;                // Ray angle cosinus multiplied by 128 for horizontal collision check (hcc).
  uint16_t hccSinBy128 = 0;               // Ray angle sinus multiplied by 128 for horizontal collision check.
  uint16_t hccTanByBlockSize = 0;         // Ray angle tangente multiplied by BLOCK_SIZE for horizontal collision check.
  int16_t hccX = 0;                       // X position of the horizontal collision check (world coordinates).
  int16_t hccY = 0;                       // Y position of the horizontal collision check (world coordinates).
  int16_t hccStepX = 0;                   // Step to make along the X axis during the horizontal collision check (world coordinates).
  int16_t hccStepY = 0;                   // Step to make along the Y axis during the horizontal collision check (world coordinates).
  uint16_t vccRayLength = worldWidth;     // Ray length of the vertical collision check ray (world coordinates).
  uint16_t hccRayLength = worldWidth;     // Ray length of the horizontal collision check ray (world coordinates).
  uint16_t rayLength = 0;                 // Ray length of the ray chosen between vertical collision check ray and horizontal collision check ray (world coordinates).
  uint8_t blockXOnMap = 0;                // X position of a block in the world map (world map coordinates). 
  uint8_t blockYOnMap = 0;                // Y position of a block in the world map (world map coordinates).
  uint8_t vccBlockType = 0;               // Type of the block hit by the vertical collision check ray. That's the block number in the world map : wall, door, etc...  
  uint8_t hccBlockType = 0;               // Type of the block hit by the horizontal collision check ray. That's the block number in the world map : wall, door, etc...
  uint8_t blockType = 0;                  // Type of the block chosen between vertical collision check hit block and horizontal collision check hit block.
  uint16_t blockHitX = 0;                 // X position of the ray in the block hit (world coordinates).
  uint16_t blockHitY = 0;                 // Y position of the ray in the block hit (world coordinates).
  uint8_t textureSliceX = 0;              // X position of the ray in the block hit texture. That's the X position of texels in the texture used by the projected slice (texture coordinates).
  uint8_t vccTextureOrient = 0;           // Tells how to render the texture of the block hit by the vertical collision check ray : from left to right or right to left (TEXTURE_ORIENT_LEFT_TO_RIGHT or TEXTURE_ORIENT_RIGHT_TO_LEFT). 
  uint8_t hccTextureOrient = 0;           // Tells how to render the texture of the block hit by the horizontal collision check ray : from left to right or right to left (TEXTURE_ORIENT_LEFT_TO_RIGHT or TEXTURE_ORIENT_RIGHT_TO_LEFT).
  uint16_t projectedSliceHeight = 0;      // Height of the projected slice (screen coordinates).
  int16_t projectedSliceY = 0;            // Y position of the projected slice. This value can be outside of the screen (screen coordinates).
  uint8_t projectedSliceRenderStartY = 0; // Y position where the slice rendering process has to start. This value is always on the screen (screen coordinates).
  uint8_t projectedSliceRenderStopY = 0;  // Y position where the slice rendering process has to stop. This value is always on the screen (screen coordinates).
  uint8_t projectedTexelY = 0;            // Y position of the projected texel (screen coordinates).
  uint8_t projectedSliceX = 0;            // X position of the projected slice (screen coordinates).
  uint16_t textureSliceRenderStepByK = 0; // Step to make inside the texture used by the projected slice. This step is multiplied by K constant in order to use integers (texture coordinates).
  uint8_t texelY = 0;                     // Y position of the texel in the texture used by the projected slice (texture coordinates).
  uint16_t texelPosInTexture = 0;         // Position of the texel bit in the texture array.
  uint8_t texelBytePosInTexture = 0;      // Position of the texel byte in the texture array.
  uint8_t texelByte = 0;                  // Texel byte read from the texture array.
  uint8_t texelPosInTexelByte = 0;        // Texel position in the texel byte read from the texture array.
  uint8_t texelByteMask = 0;              // Mask used to read texel bit from texel byte.
  uint8_t texelByteReadWithMask = 0;      // Texel byte read with the texel mask byte.
  uint8_t texel = 0;                      // Texel read from the texture : 0 or 1.
  int32_t tempLong;                       // Variable used for 24 or 32 bits operations (sometimes only...). 
  
  // Ray angle should remain between 0 and 360 degrees
  rayAngle %= 360;
  if (rayAngle < 0) rayAngle += 360;
  
  // If the ray is in the first quadrant (bottom right on the cartesian coordinate system)
  if (rayAngle >= 0 && rayAngle <= 90) {
    
    // Setup vertical collision check
    vccCosBy128 = pgm_read_byte(cosBy128 + rayAngle);
    vccSinBy128 = pgm_read_byte(cosBy128 + (90 - rayAngle)); 
    if (rayAngle == 90) vccTanByBlockSize = 0; else vccTanByBlockSize = (vccSinBy128 << MULTIPLY_BY_BLOCK_SIZE) / vccCosBy128; // tan = sin/cos with avoiding a divide by zero
    vccStepX = BLOCK_SIZE; 	
    vccStepY = vccTanByBlockSize;
    vccX = player.x + BLOCK_SIZE - (player.x & (BLOCK_SIZE - 1)); // Equals to "vccX = player.x + BLOCK_SIZE - (player.x % BLOCK_SIZE);"
    tempLong = vccX - player.x; 
    tempLong = tempLong * vccTanByBlockSize;
    vccY = player.y + (tempLong >> DIVIDE_BY_BLOCK_SIZE);
    vccTextureOrient = TEXTURE_ORIENT_LEFT_TO_RIGHT;
    
    // Setup horizontal collision check
    hccCosBy128 = pgm_read_byte(cosBy128 + (90 - rayAngle));
    hccSinBy128 = pgm_read_byte(cosBy128 + rayAngle);
    if (rayAngle == 0) hccTanByBlockSize = 0; else hccTanByBlockSize = (hccSinBy128 << MULTIPLY_BY_BLOCK_SIZE) / hccCosBy128; // tan = sin/cos with avoiding a divide by zero
    hccStepY = BLOCK_SIZE;	
    hccStepX = hccTanByBlockSize; 
    hccY = player.y + BLOCK_SIZE - (player.y & (BLOCK_SIZE - 1)); // Equals to "hccY = player.y + BLOCK_SIZE - (player.y % BLOCK_SIZE);"
    tempLong = hccY - player.y;
    tempLong = tempLong * hccTanByBlockSize;
    hccX = player.x + (tempLong >> DIVIDE_BY_BLOCK_SIZE);
    hccTextureOrient = TEXTURE_ORIENT_RIGHT_TO_LEFT;
  }
  
  // If the ray is in the second quadrant (bottom left on the cartesian coordinate system)
  else if (rayAngle > 90 && rayAngle <= 180) {
    
    // Vertical collision check setup
    vccCosBy128 = pgm_read_byte(cosBy128 + (180 - rayAngle));
    vccSinBy128 = pgm_read_byte(cosBy128 + (rayAngle - 90));
    if (rayAngle == 90) vccTanByBlockSize = 0; else vccTanByBlockSize = (vccSinBy128 << MULTIPLY_BY_BLOCK_SIZE) / vccCosBy128; // tan = sin/cos with avoiding a divide by zero
    vccStepX = -BLOCK_SIZE; 	
    vccStepY = vccTanByBlockSize;
    vccX = player.x - (player.x & (BLOCK_SIZE - 1)) - 1; // Equals to "vccX = player.x - (player.x % BLOCK_SIZE) - 1;"
    tempLong = player.x - vccX;
    tempLong = tempLong * vccTanByBlockSize;
    vccY = player.y + (tempLong >> DIVIDE_BY_BLOCK_SIZE);
    vccTextureOrient = TEXTURE_ORIENT_RIGHT_TO_LEFT;
    
    // Horizontal collision check setup
    hccCosBy128 = pgm_read_byte(cosBy128 + (rayAngle - 90));
    hccSinBy128 = pgm_read_byte(cosBy128 + (180 - rayAngle));
    if (rayAngle == 180) hccTanByBlockSize = 0; else hccTanByBlockSize = (hccSinBy128 << MULTIPLY_BY_BLOCK_SIZE) / hccCosBy128; // tan = sin/cos with avoiding a divide by zero
    hccStepY = BLOCK_SIZE;	
    hccStepX = -hccTanByBlockSize; 
    hccY = player.y + BLOCK_SIZE - (player.y & (BLOCK_SIZE - 1)); // Equals to "hccY = player.y + BLOCK_SIZE - (player.y % BLOCK_SIZE);"
    tempLong = hccY - player.y;
    tempLong = tempLong * hccTanByBlockSize;
    hccX = player.x - (tempLong >> DIVIDE_BY_BLOCK_SIZE);
    hccTextureOrient = TEXTURE_ORIENT_RIGHT_TO_LEFT; 
  }
  
  // If the ray is in the third quadrant (top left on the cartesian coordinate system)
  else if (rayAngle > 180 && rayAngle < 270) {
    
    // Vertical collision check setup
    vccCosBy128 = pgm_read_byte(cosBy128 + (rayAngle - 180));
    vccSinBy128 = pgm_read_byte(cosBy128 + (270 - rayAngle));
    if (rayAngle == 270) vccTanByBlockSize = 0; else vccTanByBlockSize = (vccSinBy128 << MULTIPLY_BY_BLOCK_SIZE) / vccCosBy128; // tan = sin/cos with avoiding a divide by zero
    vccStepX = -BLOCK_SIZE; 	
    vccStepY = -vccTanByBlockSize; 
    vccX = player.x - (player.x & (BLOCK_SIZE - 1)) - 1; // Equals to "vccX = player.x - (player.x % BLOCK_SIZE) - 1;"
    tempLong = player.x - vccX;
    tempLong = tempLong * vccTanByBlockSize;
    vccY = player.y - (tempLong >> DIVIDE_BY_BLOCK_SIZE);
    vccTextureOrient = TEXTURE_ORIENT_RIGHT_TO_LEFT;
    
    // Horizontal collision check setup
    hccCosBy128 = pgm_read_byte(cosBy128 + (270 - rayAngle));
    hccSinBy128 = pgm_read_byte(cosBy128 + (rayAngle - 180)); 
    if (rayAngle == 180) hccTanByBlockSize = 0; else hccTanByBlockSize = (hccSinBy128 << MULTIPLY_BY_BLOCK_SIZE) / hccCosBy128; // tan = sin/cos with avoiding a divide by zero   
    hccStepY = -BLOCK_SIZE;	
    hccStepX = -hccTanByBlockSize; 
    hccY = player.y - (player.y & (BLOCK_SIZE - 1)) - 1; // Equals to "hccY = player.y - (player.y % BLOCK_SIZE) - 1;"
    tempLong = player.y - hccY;
    tempLong = tempLong * hccTanByBlockSize;
    hccX = player.x - (tempLong >> DIVIDE_BY_BLOCK_SIZE);
    hccTextureOrient = TEXTURE_ORIENT_LEFT_TO_RIGHT; 
  }
  
  // If the ray is in the fourth quadrant (top right on the cartesian coordinate system)
  else { 

    // Vertical collision check setup
    vccCosBy128 = pgm_read_byte(cosBy128 + (360 - rayAngle));
    vccSinBy128 = pgm_read_byte(cosBy128 + (rayAngle - 270));
    if (rayAngle == 270) vccTanByBlockSize = 0; else vccTanByBlockSize = (vccSinBy128 << MULTIPLY_BY_BLOCK_SIZE) / vccCosBy128; // tan = sin/cos with avoiding a divide by zero
    vccStepX = BLOCK_SIZE; 	
    vccStepY = -vccTanByBlockSize; 
    vccX = player.x + BLOCK_SIZE - (player.x & (BLOCK_SIZE - 1)); // Equals to "vccX = player.x + BLOCK_SIZE - (player.x % BLOCK_SIZE);"
    tempLong = vccX - player.x;
    tempLong = tempLong * vccTanByBlockSize;
    vccY = player.y - (tempLong >> DIVIDE_BY_BLOCK_SIZE);
    vccTextureOrient = TEXTURE_ORIENT_LEFT_TO_RIGHT;

    // Horizontal collision check setup
    hccCosBy128 = pgm_read_byte(cosBy128 + (rayAngle - 270));
    hccSinBy128 = pgm_read_byte(cosBy128 + (360 - rayAngle));
    if (rayAngle == 360) hccTanByBlockSize = 0; else hccTanByBlockSize = (hccSinBy128 << MULTIPLY_BY_BLOCK_SIZE) / hccCosBy128; // tan = sin/cos with avoiding a divide by zero
    hccStepY = -BLOCK_SIZE;	
    hccStepX = hccTanByBlockSize; 
    hccY = player.y - (player.y & (BLOCK_SIZE - 1)) - 1; // Equals to "hccY = player.y - (player.y % BLOCK_SIZE) - 1;"
    tempLong = player.y - hccY;
    tempLong = tempLong * hccTanByBlockSize;
    hccX = player.x + (tempLong >> DIVIDE_BY_BLOCK_SIZE);
    hccTextureOrient = TEXTURE_ORIENT_LEFT_TO_RIGHT; 
  }
   
  // Vertical collision check
  while (vccX >= 0 && vccX < worldWidth && vccY >= 0 && vccY < worldHeight && rayAngle != 90 && rayAngle != 270) {
    
    // Get block from world map
    blockXOnMap = vccX >> DIVIDE_BY_BLOCK_SIZE;
    blockYOnMap = vccY >> DIVIDE_BY_BLOCK_SIZE;
    vccBlockType = pgm_read_byte(worldMap + blockYOnMap * worldMapWidth + blockXOnMap);

    // If the block is solid (wall, door, ...)
    if (vccBlockType > 0) {
      
      // Save ray length and stop collision check
      tempLong = vccX - player.x;
      tempLong = tempLong << MULTIPLY_BY_128;
      vccRayLength = abs(tempLong / vccCosBy128);
      break;
    }
    
    // Go to the next block
    vccX += vccStepX;
    vccY += vccStepY;
  }
  
  // Horizontal collision check
  while (hccX >= 0 && hccX < worldWidth && hccY >= 0 && hccY < worldHeight && rayAngle != 0 && rayAngle != 180) {
    
    // Get block from world map 
    blockXOnMap = hccX >> DIVIDE_BY_BLOCK_SIZE; 
    blockYOnMap = hccY >> DIVIDE_BY_BLOCK_SIZE;  
    hccBlockType = pgm_read_byte(worldMap + blockYOnMap * worldMapWidth + blockXOnMap);
    
    // If the block is solid (wall, door, ...)
    if (hccBlockType > 0) {
      
      // Save ray length and stop collision check
      tempLong = hccY - player.y;
      tempLong = tempLong << MULTIPLY_BY_128; 
      hccRayLength = abs(tempLong / hccCosBy128);
      break;
    }
    
    // Go to the next block
    hccX += hccStepX;
    hccY += hccStepY;
  }
  
  // Choose shortest ray between vertical collision check ray and horizontal collision check ray
  if (hccRayLength < vccRayLength) {
        
    rayLength = hccRayLength;
    blockHitX = hccX;
    blockHitY = hccY;
    blockType = hccBlockType;
    textureSliceX = (blockHitX & (BLOCK_SIZE - 1)) >> DIVIDE_BY_TEXTURE_SCALING_FACTOR; // Equals to "textureSliceX = (blockHitX % BLOCK_SIZE) / TEXTURE_SCALING_FACTOR;"
    if (hccTextureOrient == TEXTURE_ORIENT_RIGHT_TO_LEFT) {
      
      textureSliceX = (TEXTURE_SIZE - 1) - textureSliceX;
    }
  }
  else {
    
    rayLength = vccRayLength; 
    blockHitX = vccX;
    blockHitY = vccY;
    blockType = vccBlockType;

    textureSliceX = (blockHitY & (BLOCK_SIZE - 1)) >> DIVIDE_BY_TEXTURE_SCALING_FACTOR; // Equals to "textureSliceX = (blockHitY % BLOCK_SIZE) / TEXTURE_SCALING_FACTOR;"
    if (vccTextureOrient == TEXTURE_ORIENT_RIGHT_TO_LEFT) {
      
      textureSliceX = (TEXTURE_SIZE - 1) - textureSliceX;
    }
  }
  
  // If the current view is a 2D view
  if (view == VIEW_2D_ONERAY || view == VIEW_2D) {
    
      // Draw the current ray on the screen
      if (rayLength) {
    
        display.drawLine(player.x / WORLD_TO_SCREEN_SCALING_FACTOR, player.y / WORLD_TO_SCREEN_SCALING_FACTOR, blockHitX / WORLD_TO_SCREEN_SCALING_FACTOR, blockHitY / WORLD_TO_SCREEN_SCALING_FACTOR, 1);
      }
  }

  // If the current view is a 3D view
  else {
    
    // Apply a "Fishbowl effect correction" on the ray length (correct distance = distorted distance * cos(angle))
    tempLong = rayLength;
    tempLong = tempLong * pgm_read_byte(cosBy128 + abs(rayNumber - HALF_FOV));
    rayLength = tempLong >> DIVIDE_BY_128;
    
    // -----------------------------------------
    // Render 3D view on the screen (projection)
    // -----------------------------------------
   
    // Calculate the projected slice height
    //
    //                                                Slice height (word coordinates) * Distance to projection plane (screen coordinates)
    // Projected slice height (screen coordinates)  = ----------------------------------------------------------------------------------- 
    //                                                                   Distance to the slice (word coordinates)
    //
    // Slice height = BLOCK_SIZE = 64
    //
    // Field of view = FOV = 64
    // FOV / 2 = HALF_FOV = 32
    // Screen width = SCREEN_WIDTH = 128
    // Screen width / 2 = HALF_SCREEN_WIDTH = 64
    // HALF_SCREEN_WIDTH / tan(HALF_FOV) = 64 / tan(32) = 102 = Distance to projection plane
    //
    // Distance to the slice = rayLength
    //
    //                           BLOCK_SIZE * 102     64 * 102        6528        PROJECTION_K
    // Projected Slice Height = ------------------ = ----------- = ----------- = --------------
    //                               rayLength        rayLength     rayLength      rayLength
    //
    if (rayLength == 0) {
      
      projectedSliceHeight = PROJECTION_K;
    }
    else {
       
       projectedSliceHeight = PROJECTION_K / rayLength;
    }
    
    // Calculate the X position of the projected slice on the screen
    projectedSliceX = rayNumber << MULTIPLY_BY_2;
    
    // Calculate the Y position of the projected slice on the screen and initialize projected slice render process
    projectedSliceY = HALF_SCREEN_HEIGHT - (projectedSliceHeight >> DIVIDE_BY_2);
    if (projectedSliceY < 0) {
      
      projectedSliceRenderStartY = 0 - projectedSliceY;
      projectedSliceRenderStopY = projectedSliceRenderStartY + SCREEN_HEIGHT - 1;
    }
    else {
      
      projectedSliceRenderStartY = 0;
      projectedSliceRenderStopY = projectedSliceHeight - 1;
    }
    
    // If the view is the VIEW_3D_SOLID view
    if (view == VIEW_3D_SOLID) {
      
      // Render a solid slice
      display.drawFastVLine(projectedSliceX, projectedSliceY + projectedSliceRenderStartY, projectedSliceHeight, 1);
      display.drawFastVLine(projectedSliceX + 1, projectedSliceY + projectedSliceRenderStartY, projectedSliceHeight, 1);
    }
    
    // If the view is the VIEW_3D_TEXTURED view
    else {
      
      // -----------------------
      // Render a textured slice
      // -----------------------
      
      // Calculate texture slice render step
      //
      //                                  Texture height 
      // Texture slice render step = ------------------------ 
      //                              Projected slice height
      //
      //                                  Texture height * k             TEXTURE_SIZE * K                 32 * 128                TEXTURE_SIZE_BY_K
      // Texture slice render step = ---------------------------- = -------------------------- = -------------------------- = --------------------------
      //                              Projected slice height * k     projectedSliceHeight * K     projectedSliceHeight * K     projectedSliceHeight * k
      //
      //                                   TEXTURE_SIZE_BY_K
      // Texture slice render step * k = ----------------------
      //                                  projectedSliceHeight
      //
      textureSliceRenderStepByK = TEXTURE_SIZE_BY_K / projectedSliceHeight;
      
      // Render the textured slice on the screen
      for (uint8_t projectedSliceRenderY = projectedSliceRenderStartY; projectedSliceRenderY <= projectedSliceRenderStopY; projectedSliceRenderY++) {
          
        // Get pixel from the texture (get texel)
        texelY = (projectedSliceRenderY * textureSliceRenderStepByK) >> DIVIDE_BY_K;
        texelPosInTexture = texelY * TEXTURE_SIZE + textureSliceX;
        texelBytePosInTexture = texelPosInTexture >> DIVIDE_BY_8;
        texelByte = pgm_read_byte(texturesArray[blockType - 1] + texelBytePosInTexture); 
        texelPosInTexelByte = texelPosInTexture & 7; // Equals to texelPosInTexture % 8
        texelByteMask = 128 >> texelPosInTexelByte;
        texelByteReadWithMask = texelByte & texelByteMask;
        texel = texelByteReadWithMask >> (7 - texelPosInTexelByte);
        
        // Draw the texel
        projectedTexelY = projectedSliceY + projectedSliceRenderY;
        display.drawPixel(projectedSliceX, projectedTexelY, texel);
        display.drawPixel(projectedSliceX + 1, projectedTexelY, texel);          
      } 
    }
  }
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Load a given world map in the engine.
// ------------------------------------------------------------------------------------------------------------------------------------------------------
void ARCE::loadWorldMap(const uint8_t *worldMap, uint8_t worldMapWidth, uint8_t worldMapHeight) {
  
  this->worldMap = worldMap;
  this->worldMapWidth = worldMapWidth;
  this->worldMapHeight = worldMapHeight;
  worldWidth = worldMapWidth * BLOCK_SIZE; 
  worldHeight = worldMapHeight * BLOCK_SIZE;
}

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Read a pixel from a given texture
// ------------------------------------------------------------------------------------------------------------------------------------------------------
uint8_t ARCE::getTexel (uint8_t texelX, uint8_t texelY, const uint8_t *texture, uint8_t textureWidth, uint8_t textureHeight) {
  
  uint16_t texelPosInTexture = 0;    // Position of the texel bit in the texture array.
  uint8_t texelBytePosInTexture = 0; // Position of the texel byte in the texture array.
  uint8_t texelByte = 0;             // Texel byte read from the texture array.
  uint8_t texelPosInTexelByte = 0;   // Texel position in the texel byte read from the texture array.
  uint8_t texelByteMask = 0;         // Mask used to read texel bit from texel byte.
  uint8_t texelByteReadWithMask = 0; // Texel byte read with the texel mask byte.
  uint8_t texel = 0;                 // Texel read from the texture : 0 or 1.

  texelPosInTexture = texelY * textureWidth + texelX;
  texelBytePosInTexture = texelPosInTexture >> DIVIDE_BY_8;
  texelByte = pgm_read_byte(texture + texelBytePosInTexture); 
  texelPosInTexelByte = texelPosInTexture & 7; // equals to texelPosInTexture % 8
  texelByteMask = 128 >> texelPosInTexelByte;
  texelByteReadWithMask = texelByte & texelByteMask;
  texel = texelByteReadWithMask >> (7 - texelPosInTexelByte);
  
  return texel;
}

