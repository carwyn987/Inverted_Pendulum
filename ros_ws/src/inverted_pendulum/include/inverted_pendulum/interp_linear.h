/*
https://github.com/blackstonep/Numerical-Recipes/tree/master
Sourced from blackstonep, @gwr69 - gwr69 Guido Wolf Reichert 

Originally from Numerical Recipes in C++: The Art of Scientific Computing (William H. Press, Brian P. Flannery, Saul A. Teukolsky, William T. Vetterling; New York: Cambridge University Press, 2002 (2nd ed., p. 113-114))
*/

// struct Linear_interp : Base_interp
// {
// 	Linear_interp(VecDoub_I &xv, VecDoub_I &yv)
// 		: Base_interp(xv,&yv[0],2)  {}
// 	Doub rawinterp(Int j, Doub x) {
// 		if (xx[j]==xx[j+1]) return yy[j];
// 		else return yy[j] + ((x-xx[j])/(xx[j+1]-xx[j]))*(yy[j+1]-yy[j]);
// 	}
// };
