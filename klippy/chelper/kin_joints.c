#include <stdio.h> // malloc
#include <stdlib.h> // malloc
#include <stddef.h> // offsetof
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "list.h" // container_of
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_coord

struct joints_stepper {
    struct stepper_kinematics sk;
    char axis;
};

static double
joints_calc_position(struct stepper_kinematics *sk, struct move *m, double move_time)
{
    struct joints_stepper *ds = container_of(sk, struct joints_stepper, sk);
    struct coord coord = move_get_coord(m, move_time);

    switch (ds->axis) {
        case 'x': return coord.x;
        case 'y': return coord.y;
        case 'z': return coord.z;
        case 'a': return coord.a;
        case 'b': return coord.b;
        case 'c': return coord.c;
    }
    abort();
}

struct stepper_kinematics * __visible
joints_stepper_alloc(char axis)
{
    struct joints_stepper *ds = malloc(sizeof(*ds));
    memset(ds, 0, sizeof(*ds));
	ds->sk.calc_position_cb = joints_calc_position;
    ds->axis = axis;
    ds->sk.active_flags = AF_X | AF_Y | AF_Z | AF_A | AF_B | AF_C;
    return &ds->sk;
}
