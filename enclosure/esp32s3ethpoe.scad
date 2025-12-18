// esp32s3 eth poe
$c_p=5;

width = 24;
height = 28;
length = 80;
rad = 3;
th = 2;

ew=17;
eh=13.5;
el=16;

pw = 22;
pth = 2.5;
pl = 73;


translate([0,0,0+length-40+10])
    //mirror([0,0,1])
    endcap();
//lipstick();

// endcap
module endcap() {
    difference (){
        union() {
            //flat plate
            rounded_square(width, height, th/2, rad);
            // inside shape
            translate([0,0,0+th*1.5])
                rounded_square(width-th-.4, height-th-.4, th*3.5, rad);

            //bump for holding
            translate([0-3,0+width/2-th/2,0+th*2])
                rotate([0,90,0])
                    cylinder(6, 1, 1, $fn=10);
            translate([0-3,0-width/2+th/2,0+th*2])
                rotate([0,90,0])
                    cylinder(6, 1, 1, $fn=10);
            
        }
        translate([0,0,0+th*2.5+.1])
            rounded_square(width-th*2, height-th*2, th*4, rad);
        
    }
}

module lipstick() {
    union() {
        difference() {    
            union() {
                rounded_square(width, height, length, rad);


            }
            // hollow main
            translate([0,0,0+th+4])
                rounded_square(width-2, height-2, length, rad);
            // clear usbc connector
            translate([0+th*3.5,0,0-length/2+th*4-1])
                rounded_square(width-5, 12, 12, rad);
            
            // pcb
            translate([0+th*3,0,0-th-1])
                cube([pth, pw, pl], true);
            
            // ethernet jack
            translate([0-th,0,0+th-length/2])
                cube([eh, ew, el], true);
            
            // usb jack
            translate([0+th*4+.5,0,0-length/2-th])
                rounded_strut(6, 4, 10, 0);
            
            // rear clip holes
            translate([0,0-20,0+length/2-th*2])
                rotate([0,90,90])
                rounded_strut(6, 2, 50, 0);
            
        }

        // pcb
    //    translate([0+th*3,0,0-th-1])
    //        cube([pth, pw, pl], true);

        // rear pcb guide bump
        translate([0+th*1.5,0-width/2+th/2,0+length/2-th*6])
            rotate([90,0,90])
            rounded_strut(6, 2, 2, 0);
    //    // rear pcb guide bump
    //    translate([0+th*3.5,0-width/2+th/2,0+length/2-th*6])
    //        rotate([90,0,90])
    //        rounded_strut(6, 2, 2, 0);
        
    //    // rear pcb guide bump
    //    translate([0+th*1.5,0+width/2-th/2,0+length/2-th*6])
    //        rotate([90,0,90])
    //        rounded_strut(6, 2, 2, 0);
        // rear pcb guide bump
        translate([0+th*3.5,0+width/2-th/2,0+length/2-th*6])
            rotate([90,0,90])
            rounded_strut(6, 2, 2, 0);

    }
    
}


module rounded_strut($length, $width, $thickness, $hole_dia) {
    translate([0+$width/2,0,0+$thickness/2])
        difference(){
            union(){
                translate([0-$width/2,0+$length/2,0])
                    cylinder($thickness, $width/2, $width/2, $fn=$width*$c_p, true);
                translate([0-$width/2,0-$length/2,0])
                    cylinder($thickness, $width/2, $width/2, $fn=$width*$c_p, true);
                translate([0-$width/2,0,0])
                    cube([$width, $length,$thickness], true);
            }
            translate([0-$width/2,0+$length/2,0-0.1])
                cylinder($thickness+0.5, $hole_dia/2, $hole_dia/2, $fn=$hole_dia*$c_p, true);
            translate([0-$width/2,-$length/2,0-0.1])
                cylinder($thickness+0.5, $hole_dia/2, $hole_dia/2, $fn=$hole_dia*$c_p, true);

        }
}

module rounded_corner(rch, rcr, rth) {
    difference(){
        tube(rch, rcr, rcr, rcr-rth, rcr-rth, $fn=rcr*$c_p);
        translate([-1,0-rcr,-1])
            cube([rcr*2, rcr, rch*2]);
        
        translate([0-rcr,0-rcr,-1])
            cube([rcr, rcr*2, rch*2]);
    }
    translate([0+rcr/2-rth/2,0-rth/20,0])
        cube([rth/2, rth/20, rch]);
    translate([0-rth/20,0+rcr/2-rth/2,0])
        cube([rth/20, rth/2, rch]);
}


module rounded_square(ch_l, ch_w, ch_h, ch_c_rad) {
    // Square cross
    translate([0+ch_c_rad,0,0])
        cube([ch_w-ch_c_rad*2,ch_l-ch_c_rad*2,ch_h], true);
    translate([0-ch_c_rad,0,0])
        cube([ch_w-ch_c_rad*2,ch_l-ch_c_rad*2,ch_h], true);
    translate([0,0+ch_c_rad/2,0])
        cube([ch_w-ch_c_rad*2,ch_l-ch_c_rad,ch_h], true);
    translate([0,0-ch_c_rad/2,0])
        cube([ch_w-ch_c_rad*2,ch_l-ch_c_rad,ch_h], true);

    // Rounded corners
    translate([0-ch_w/2+ch_c_rad,0-ch_l/2+ch_c_rad,0])
        cylinder(ch_h, ch_c_rad, ch_c_rad, $fn=50, true);
    translate([0-ch_w/2+ch_c_rad,0+ch_l/2-ch_c_rad,0])
        cylinder(ch_h, ch_c_rad, ch_c_rad, $fn=50, true);
    translate([0+ch_w/2-ch_c_rad,0+ch_l/2-ch_c_rad,0])
        cylinder(ch_h, ch_c_rad, ch_c_rad, $fn=50, true);
    translate([0+ch_w/2-ch_c_rad,0-ch_l/2+ch_c_rad,0])
        cylinder(ch_h, ch_c_rad, ch_c_rad, $fn=50, true);
    
}


module profile_rounded_corners2 (cw, cl, ch, cth, cdia){
    // width, length, height, thickness, radius
    difference(){
        hull() {
            translate([0-cw/2+cdia/2,0-cl/2+cdia/2,0])
                cylinder(ch, cdia/2, cdia/2, $fn=cdia*$c_p, true);
            translate([0+cw/2-cdia/2,0+cl/2-cdia/2,0])
                cylinder(ch, cdia/2, cdia/2, $fn=cdia*$c_p, true);
            translate([0+cw/2-cdia/2,0-cl/2+cdia/2,0])
                cylinder(ch, cdia/2, cdia/2, $fn=cdia*$c_p, true);
            translate([0-cw/2+cdia/2,0+cl/2-cdia/2,0])
                cylinder(ch, cdia/2, cdia/2, $fn=cdia*$c_p, true);
        }
        hull() {
            translate([0-cw/2+cdia/2+cth*2,0-cl/2+cdia/2+cth*2,0])
                cylinder(ch, cdia/2, cdia/2, $fn=cdia*$c_p, true);
            translate([0+cw/2-cdia/2-cth*2,0+cl/2-cdia/2-cth*2,0])
                cylinder(ch, cdia/2, cdia/2, $fn=cdia*$c_p, true);
            translate([0+cw/2-cdia/2-cth*2,0-cl/2+cdia/2+cth*2,0])
                cylinder(ch, cdia/2, cdia/2, $fn=cdia*$c_p, true);
            translate([0-cw/2+cdia/2+cth*2,0+cl/2-cdia/2-cth*2,0])
                cylinder(ch, cdia/2, cdia/2, $fn=cdia*$c_p, true);
        }
    }    
}

module tube($height, $od, $od2, $id, $id2, $fn){
    difference(){
//        cylinder($height, $od/2, $od2/2, $fn=$od*$c_p);
        cylinder($height, $od/2, $od2/2, $fn=$fn);
        translate([0,0,-1])
//            cylinder($height+2, $id/2, $id2/2, $fn=$od*$c_p);
            cylinder($height+2, $id/2, $id2/2, $fn=$fn);
    }
}
