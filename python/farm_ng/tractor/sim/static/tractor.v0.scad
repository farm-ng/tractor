wheel_base=46.25*25.4;
wheel_radius=(15/2.0)*25.4;
wheel_width=(4.75)*25.4;
caster_height=10*25.4;
x_length=24*25.4;
gps_height=41*25.4;
square_tube_w=1.5*25.4;
gps_x_offset=4*25.4;

module wheel(radius, width) {
    translate([0,0,radius])
    rotate(90, [1,0,0])
    cylinder(h = width, r1=radius, r2 = radius, center = true);
}

module caster(height) {
    translate([0,0,height/2])
    cylinder(h = height, r1=height/2, r2 = height/2, center = true);
}

module side_assembly(left_or_right=1) {
    translate([x_length,0,0])
    caster(caster_height);

    translate([x_length/2.0,100,caster_height + 0.5*square_tube_w])
    cube([x_length, square_tube_w, square_tube_w], center=true);

    translate([x_length/2.0,-100,caster_height + 0.5*square_tube_w])
    cube([x_length, square_tube_w, square_tube_w], center=true);

    vertical_bar_height = (gps_height-wheel_radius);
    translate([gps_x_offset,left_or_right*100,wheel_radius + 0.5*vertical_bar_height])
    cube([square_tube_w, square_tube_w, vertical_bar_height], center=true);

    wheel(wheel_radius, wheel_width);
}



rotate(180,[0,0,1])
translate([0,0,0]) {
    translate([0,wheel_base/2,0])
    side_assembly(-1);

    translate([0,-wheel_base/2,0])
    side_assembly(1);

    translate([gps_x_offset,0,gps_height - square_tube_w/2])
    cube([square_tube_w, wheel_base, square_tube_w], center=true);

    translate([gps_x_offset,0, gps_height+10])
    cube([80,80,20],center=true);
}
