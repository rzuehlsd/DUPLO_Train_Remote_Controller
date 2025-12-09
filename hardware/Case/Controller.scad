/**
* MachineBlocks
* https://machineblocks.com/examples/boxes-enclosures
*
* Channel 12x3
* Copyright (c) 2022 - 2025 Jan Philipp Knoeller <pk@pksoftware.de>
*
* Published under license:
* Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International 
* https://creativecommons.org/licenses/by-nc-sa/4.0/
*
*/
use <MachineBlocks/lib/block.scad>;

/* [View] */
// How to view the brick in the editor
viewMode = "print"; // [print, assembled, cover]

/* [Size] */

// Box size in X-direction specified as multiple of an 1x1 brick.
boxSizeX = 16; // [1:32] 
// Box size in Y-direction specified as multiple of an 1x1 brick.
boxSizeY = 5; // [1:32] 
// Total box height specified as number of layers. Each layer has the height of one plate.
boxLayers = 4; // [1:24]

/* [Appearance] */

// Type of cut-out on the underside.
baseCutoutType = "classic"; // [none, classic]
// Whether the base should have knobs
baseKnobs = false;
// Type of the base knobs
baseKnobType = "classic"; // [classic, technic]
// Whether base knobs should be centered.
baseKnobCentered = false;
// Whether the pit should contain knobs
basePitKnobs = false;
// Type of the base pit knobs
basePitKnobType = "classic"; // [classic, technic]
// Whether base pit knobs should be centered.
basePitKnobCentered = false;
// Pit wall thickness
basePitWallThickness = 0.333;
// Pit wall gaps
basePitWallGaps = [[0, 0, 0], [0, 0, 0]];
// Whether the base should have a tongue
baseTongue = true;

// Whether the box should have a lid
lid = true;
// Lid height specified as number of layers. Each layer has the height of one plate.
lidLayers = 1; // [1:24]
// Whether the lid should have knobs
lidKnobs = false;
// Type of the lid knobs
lidKnobType = "classic"; // [classic, technic]
// Whether lid knobs should be centered.
lidKnobCentered = false;
// Whether lid should have pillars
lidPillars = true;
// Whether lid should be permanent (non removable)
lidPermanent = true;

/* [Quality] */

// Quality of the preview in relation to the final rendering.
previewQuality = 0.5; // [0.1:0.1:1]
// Number of drawn fragments for roundings in the final rendering.
roundingResolution = 64; // [16:8:128]

/* [Calibration] */

//Adjustment of the height (mm)
baseHeightAdjustment = 0.0;
//Adjustment of each side (mm)
baseSideAdjustment = -0.1;
//Diameter of the knobs (mm)
knobSize = 5.0;
//Thickness of the walls (mm)
wallThickness = 1.5;
//Diameter of the Z-Tubes (mm)
tubeZSize = 6.4;


// Koordinatenpositionen vom Ursprungin Mitte des Deckels
l = 127.5;  // Laenge 16 Noppen aussen
b = 39.7;   // Breite 5 Noppen aussen
delta = 1;  // Abstand vom Innenwand

function toX0(x) = l/2 - wallThickness - delta - x;


// Alle x Positionen von oberer Kante

// Drehencoder 
enc_x = toX0(8.5); 
enc_y = 0;       
enc_r = 3.5;

// RGB LED
led_x = toX0(27.0);
led_y = 0;
led_r = 1;

// Ausbruch für Knöpfe
btn_dx = 15;
btn_dy = 15;


// Button (1,1)  Record
btn1_x = toX0(53.6);
btn1_y = 10;

// Button (1,2)  Play
btn2_x = toX0(53.6);
btn2_y = btn1_y - 20 ;

// Button (2,1)  Stop
btn3_x = toX0(53.6 + 20);
btn3_y = btn1_y ;

// Button (2,2)  LED
btn4_x = toX0(53.6 + 20);
btn4_y = btn1_y - 20;

// Button (3,1)  Water
btn5_x = toX0(53.6 + 2 * 20);
btn5_y = btn1_y;

// Button (3,2)  Sound
btn6_x = toX0(53.6 + 2 * 20);
btn6_y = btn1_y - 20;

// USB Typ C links
usb_x = toX0(31.5);  // mitte          
usb_z = 9.5;
usb_dz = 5;
usb_dx = 12;



module makeHole(x,y,r)
{
    echo("makeHole: Variable x = ", x);
    echo("Variable y = ", y);
    color([1,0,0])
        translate([x, y, 1])
            cylinder(10,r,r,true);
}


module makeRectHole(x,y,dx,dy)
{
    echo("makeRectHole: Variable x = ", x);
    echo("Variable y = ", y);
    color([1,0,0])
        translate([x, y, 0])
            cube([dx, dy, 10], center=true);
}


module makeLeftHole(x,z,dx,dz)
{
    echo("makeLeftHole: Variable x = ", x);
    echo("Variable z = ", z, "dz =", dz);
    color([1,0,0])
        translate([x, 18, z])
            cube([dx, 5, dz], center = true);
}


// Druckknöpfe
module button(farbe)
{
    color(farbe)
        {
            cube([16, 16, 2], center = true);
            translate([0,0,4])
                cube([14.5, 14.5, 8], center = true);
        }
}


module ButtonRecord(farbe){
    union(){
        button(farbe);
        translate([0,0,8])
            color([1,0,0])
                cylinder(1,3,3,true);
    }
}

module ButtonPlay(farbe){
    union(){
        button(farbe);
        translate([0,0,8])
            color([1,1,1])
                cylinder(1,4,4,true, $fn=3);
    }
}

// Knopf für Drehencoder
module Knob(farbe)
{
    height = 13;
    r1 = 12;
    r2 = 10;
    eps = 0.1;   // Klemmung
    w_wall = 2.5;
    r_hole = 3 - eps;
    
    color(farbe)
        {
            difference(){
                cylinder(height,r1,r2,true);
                translate([0,0,-2])
                    cylinder(height,r1-w_wall,r2-w_wall,true);
            }
            difference(){
                cylinder(height, r_hole + w_wall, r_hole + w_wall, true);
                translate([0,0,-w_wall])
                   cylinder(height,r_hole,r_hole, true);
            }
    }
}




module case() {
    color("yellow");
    difference() {
        block(
            grid=[boxSizeX, boxSizeY],
            baseLayers = boxLayers - (lid ? lidLayers : 0),
            baseCutoutType = baseCutoutType,

            knobs = baseKnobs,
            knobType = baseKnobType,
            knobCentered = baseKnobCentered,
            
            pit=true,
            //pitWallGaps = basePitWallGaps, //boxType != "box" ? (boxType == "channel_corner" ? [ [ 0, 0, 0 ], [ 2, 0, 0 ] ] : [ [ 0, 0, 0 ], [ 1, 0, 0 ] ]) : [],
            pitWallThickness = basePitWallThickness,
            pitKnobs = basePitKnobs,
            pitKnobType = basePitKnobType,
            pitKnobCentered = basePitKnobCentered,

            tongue = baseTongue,
            tongueHeight = lidPermanent ? 2.0 : 1.8,
            tongueClampThickness = lidPermanent ? 0.1 : 0,
            tongueOuterAdjustment = lidPermanent ? 0.0 : -0.1,
            tongueRoundingRadius = lidPermanent ? 0.0 : 0.4,
            
            baseHeightAdjustment = baseHeightAdjustment,
            baseSideAdjustment = baseSideAdjustment,
            knobSize = knobSize,
            wallThickness = wallThickness,
            tubeZSize = tubeZSize
        );
        makeLeftHole(usb_x,usb_z,usb_dx,usb_dz);
    }
}

module lid() {
    color("yellow");
    translate(viewMode != "print" ? [0, 0, ((boxLayers - lidLayers) + (viewMode == "cover" ? 2*lidLayers : 0)) * 3.2] : [boxSizeX > boxSizeY ? 0 : (boxSizeX + 0.5) * 8.0, boxSizeX > boxSizeY ? -(boxSizeY + 0.5) * 8.0 : 0, 0])
    {
            difference() {
            block(
                grid=[boxSizeX, boxSizeY],
                baseLayers = lidLayers,
                baseCutoutType = lidPermanent ? "groove" : "classic",

                knobs = lidKnobs,
                knobType = lidKnobType,
                knobCentered = lidKnobCentered,

                pillars = lidPillars,
                //pitWallGaps = basePitWallGaps, //boxType != "box" ? (boxType == "channel_corner" ? [ [ 0, 0, 0 ], [ 2, 0, 0 ] ] : [ [ 0, 0, 0 ], [ 1, 0, 0 ] ]) : [],

                baseHeightAdjustment = baseHeightAdjustment,
                baseSideAdjustment = baseSideAdjustment,
                knobSize = knobSize,
                wallThickness = wallThickness,
                tubeZSize = tubeZSize
            );
            makeHole(enc_x, enc_y, enc_r);
            makeHole(led_x, led_y, led_r);
            makeRectHole(btn1_x, btn1_y, btn_dx, btn_dy);
            makeRectHole(btn2_x, btn2_y, btn_dx, btn_dy);
            makeRectHole(btn3_x, btn3_y, btn_dx, btn_dy);
            makeRectHole(btn4_x, btn4_y, btn_dx, btn_dy);
            makeRectHole(btn5_x, btn5_y, btn_dx, btn_dy);
            makeRectHole(btn6_x, btn6_y, btn_dx, btn_dy);
        }
    }
}



translate([50,50,0]) 
    Knob("red");
translate([0,50,0])
    ButtonRecord("black");
translate([-30,50,0])
    ButtonPlay("black");

// Stop Button
translate([-60,50,0])
    button("red");

// Water Button
translate([-90,50,0])
    button("blue");

// LED Button
translate([-120,50,0])
    button("white");
    
// Sound Button
translate([-150,50,0])
    button("yellow");

// Unterteil
case();

// Deckel
if(lid){
    lid();
}