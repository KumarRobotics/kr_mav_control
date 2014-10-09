#ifndef TRAJECTORY_BASE_H
#define TRAJECTORY_BASE_H



class TrajSection
{
public:
	virtual TrajSection(uint k_r_, decimal_t dt_);
	virtual ~TrajSection();

	virtual evaluate(decimal_t t, uint derr , Vec4 &out)=0;
protected:
	uint k_r; // order being minimized i.e. 4 = snap
	bool generated; // whether or not trajectory has been generated
	decimal_t dt; // durration of segment
};

class TrajGen {
public:
protected:
	
};



#endif