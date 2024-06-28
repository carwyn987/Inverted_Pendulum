// C++ includes
#include <string>
#include <iostream>
// #include "nr3.h"
// #include "interp_1d.h"
// #include "interp_linear.h"

#include <cstdlib>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

float savitzky_golay_interpolate() {
    return 0.0;
}

// int linear_interp(){
//     double tmpx[5] = {1,2,3,4,5};
//     VecDoub x(5, tmpx);
//     double tmpy[5] = {1.0,1.5,2.0,2.5,3.0};
//     VecDoub y(5, tmpy);
//     Linear_interp interp(x, y);
//     Doub interpx = 6.5;
//     Doub z = interp.interp(interpx);
//     std::cout << "Linear interpolant at " << interpx << " is " << z << std::endl;
//     return 0;
// }

float first_difference(boost::circular_buffer<float> &buffer, boost::circular_buffer<double> &time_buffer){
    return static_cast<float>(buffer[1] - buffer[0]) / (time_buffer[1] - time_buffer[0]);
}