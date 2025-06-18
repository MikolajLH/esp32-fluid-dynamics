#ifndef SIMULATION_GUARD
#define SIMULATION_GUARD

class Simulation
{
public:
	static constexpr int Nx = 64;
	static constexpr int Ny = 32;
	static constexpr int Rs = Ny + 2;
	static constexpr int Cs = Nx + 2;
	static constexpr int SIZE = Rs * Cs;
	static constexpr int DIFFUSE_ITER = 20;
	static constexpr int PROJECT_ITER = 30;

	static constexpr float minX = 1.f;
	static constexpr float maxX = float(Nx);
	static constexpr float minY = 1.f;
	static constexpr float maxY = float(Ny);

	static constexpr int IX(int x, int y) { return Cs * y + x; }


	Simulation();
	~Simulation();

	void reset();
	void clear_buffer();
	void set_bnd(float* arr, int flag);
	void project();
	void advect(float* value, float* value0, float* u, float* v, int flag);
	void diffuse(float* value, float* value0, float rate, int flag);
	void vort_confinement();
	void add_source();
	void vel_step();
	void den_step();


	float* vx;
	float* vy;
	float* vx0;
	float* vy0;
	float* d;
	float* d0;

	//vorticity confinement
	float* vort;
	float* gradVortX;
	float* gradVortY;
	float* lenGrad;

	float visc;
	float diff;
	float vorticity;
	float dt;
};
#endif