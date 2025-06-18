#include <stdlib.h>
#include <utility>
#include <math.h>

#include "Simulation.h"

Simulation::Simulation() : 
	vx {  (float*)malloc(sizeof(float) * SIZE) },
	vy {  (float*)malloc(sizeof(float) * SIZE) },
	vx0{  (float*)malloc(sizeof(float) * SIZE) },
	vy0{  (float*)malloc(sizeof(float) * SIZE) },
	d  {  (float*)malloc(sizeof(float) * SIZE) },
	d0 {  (float*)malloc(sizeof(float) * SIZE) }, 

	vort     {  (float*)malloc(sizeof(float) * SIZE) },
	gradVortX{  (float*)malloc(sizeof(float) * SIZE) },
	gradVortY{  (float*)malloc(sizeof(float) * SIZE) },
	lenGrad  {  (float*)malloc(sizeof(float) * SIZE) },

	visc{ 0.1f },
	diff{ 0.1f },
	vorticity{ 0.1f },
	dt{ 0.016f } {
}

Simulation::~Simulation() {
	free(vx);
	free(vy);
	free(vx0);
	free(vy0);
	free(d);
	free(d0);

	free(vort);
	free(gradVortX);
	free(gradVortY);
	free(lenGrad);
}

void Simulation::reset() {
	for (int i = 0; i < SIZE; i++)
	{
		vx[i] = 0.0f;
		vy[i] = 0.0f;
		d[i] = 0.0f;
	}
	clear_buffer();
}

void Simulation::clear_buffer() {
	for (int i = 0; i < SIZE; i++)
	{
		vx0[i] = 0.0f;
		vy0[i] = 0.0f;
		d0[i] = 0.0f;
	}
}
void Simulation::add_source() {
	for (int i = 1; i <= Nx; i++)
	{
		for (int j = 1; j <= Ny; j++)
		{
			const int index = IX(i, j);
			vx[index] += vx0[index];
			vy[index] += vy0[index];
			d[index] += d0[index];
		}
	}

	set_bnd(vx, 1);
	set_bnd(vy, 2);
	set_bnd(d, 0);
}

void Simulation::advect(float* value, float* value0, float* u, float* v, int flag) {
	float oldX;
	float oldY;
	int i0;
	int i1;
	int j0;
	int j1;
	float wL;
	float wR;
	float wB;
	float wT;

	for (int i = 1; i <= Nx; i++)
	{
		for (int j = 1; j <= Ny; j++)
		{
			oldX = ((float)i + 0.5f) - u[IX(i, j)] * dt;
			oldY = ((float)j + 0.5f) - v[IX(i, j)] * dt;

			if (oldX < minX) oldX = minX;
			if (oldX > maxX) oldX = maxX;
			if (oldY < minY) oldY = minY;
			if (oldY > maxY) oldY = maxY;

			i0 = (int)(oldX - 0.5f);
			j0 = (int)(oldY - 0.5f);
			i1 = i0 + 1;
			j1 = j0 + 1;

			wL = ((float)i1 + 0.5f) - oldX;
			wR = 1.0f - wL;
			wB = ((float)j1 + 0.5f) - oldY;
			wT = 1.0f - wB;

			value[IX(i, j)] = wB * (wL * value0[IX(i0, j0)] + wR * value0[IX(i1, j0)]) + wT * (wL * value0[IX(i0, j1)] + wR * value0[IX(i1, j1)]);
		}
	}

	set_bnd(value, flag);
}

void Simulation::den_step() {
	if (visc > 0.0f)
	{
		std::swap(d0, d);
		diffuse(d, d0, visc, 0);
	}
	std::swap(d0, d);
	advect(d, d0, vx, vy, 0);
}

void Simulation::diffuse(float* value, float* value0, float rate, int flag) {
	for (int i = 0; i < SIZE; i++) value[i] = 0.0f;
	const float a = rate * dt;

	for (int k = 0; k < DIFFUSE_ITER; k++)
	{
		for (int i = 1; i <= Nx; i++)
		{
			for (int j = 1; j <= Ny; j++)
			{
				value[IX(i, j)] = (value0[IX(i, j)] + a * (value[IX(i + 1, j)] + value[IX(i - 1, j)] + value[IX(i, j + 1)] + value[IX(i, j - 1)])) / (4.0f * a + 1.0f);
			}
		}
		set_bnd(value, flag);
	}
}

void Simulation::project() {
	for (int i = 1; i <= Nx; i++)
	{
		for (int j = 1; j <= Ny; j++)
		{
			/*div*/vx0[IX(i, j)] = 0.5f * (vx[IX(i + 1, j)] - vx[IX(i - 1, j)] + vy[IX(i, j + 1)] - vy[IX(i, j - 1)]);
			/*p*/vy0[IX(i, j)] = 0.0f;;
		}
	}

	/*set_bnd(div, 0);
	set_bnd(p, 0);*/
	set_bnd(vx0, 0);
	set_bnd(vy0, 0);

	//projection iteration
	for (int k = 0; k < PROJECT_ITER; k++)
	{
		for (int i = 1; i <= Nx; i++)
		{
			for (int j = 1; j <= Ny; j++)
			{
				vy0[IX(i, j)] = (vy0[IX(i + 1, j)] + vy0[IX(i - 1, j)] + vy0[IX(i, j + 1)] + vy0[IX(i, j - 1)] - vx0[IX(i, j)]) / 4.0f;
			}
		}
		set_bnd(vy0, 0);
	}

	//velocity minus grad of Pressure
	for (int i = 1; i <= Nx; i++)
	{
		for (int j = 1; j <= Ny; j++)
		{
			vx[IX(i, j)] -= 0.5f * (vy0[IX(i + 1, j)] - vy0[IX(i - 1, j)]);
			vy[IX(i, j)] -= 0.5f * (vy0[IX(i, j + 1)] - vy0[IX(i, j - 1)]);
		}
	}

	set_bnd(vx, 1);
	set_bnd(vy, 2);
}

void Simulation::set_bnd(float* arr, int flag) {
	// velocity x
	if (flag == 1) {
		for (int i = 1; i <= Nx; i++)
		{
			arr[IX(i, 0)] = arr[IX(i, 1)];
			arr[IX(i, Ny + 1)] = arr[IX(i, Ny)];
		}

		for (int j = 1; j <= Ny; j++)
		{
			arr[IX(0, j)] = -arr[IX(1, j)];
			arr[IX(Nx + 1, j)] = -arr[IX(Nx, j)];
		}
	}
	// velocity y
	if (flag == 2) {

		for (int i = 1; i <= Nx; i++)
		{
			arr[IX(i, 0)] = -arr[IX(i, 1)];
			arr[IX(i, Ny + 1)] = -arr[IX(i, Ny)];
		}

		for (int j = 1; j <= Ny; j++)
		{
			arr[IX(0, j)] = arr[IX(1, j)];
			arr[IX(Nx + 1, j)] = arr[IX(Nx, j)];
		}
	}
	//density
	if (flag == 0) {
		for (int i = 1; i <= Nx; i++)
		{
			arr[IX(i, 0)] = arr[IX(i, 1)];
			arr[IX(i, Ny + 1)] = arr[IX(i, Ny)];
		}
		for (int j = 1; j <= Ny; j++)
		{
			arr[IX(0, j)] = arr[IX(1, j)];
			arr[IX(Nx + 1, j)] = arr[IX(Nx, j)];
		}
	}
	//corners
	arr[IX(0, 0)] = (arr[IX(0, 1)] + arr[IX(1, 0)]) / 2.f;
	arr[IX(Nx + 1, 0)] = (arr[IX(Nx, 0)] + arr[IX(Nx + 1, 1)]) / 2.f;
	arr[IX(0, Ny + 1)] = (arr[IX(0, Ny)] + arr[IX(1, Ny + 1)]) / 2.f;
	arr[IX(Nx + 1, Ny + 1)] = (arr[IX(Nx, Ny + 1)] + arr[IX(Nx + 1, Ny)]) / 2.f;
}

void Simulation::vel_step() {

	if (diff > 0.0f)
	{
		std::swap(vx0, vx);
		std::swap(vy0, vy);
		diffuse(vx, vx0, diff, 1);
		diffuse(vy, vy0, diff, 2);
	}

	project();

	std::swap(vx0, vx);
	std::swap(vy0, vy);
	advect(vx, vx0, vx0, vy0, 1);
	advect(vy, vy0, vx0, vy0, 2);

	project();
}

void Simulation::vort_confinement() {
	for (int i = 1; i <= Nx; i++)
	{
		for (int j = 1; j <= Ny; j++)
		{
			vort[IX(i, j)] = 0.5f * (vy[IX(i + 1, j)] - vy[IX(i - 1, j)] - vx[IX(i, j + 1)] + vx[IX(i, j - 1)]);
		}
	}
	set_bnd(vort, 0);
	
	for (int i = 1; i <= Nx; i++)
	{
		for (int j = 1; j <= Ny; j++)
		{
			gradVortX[IX(i, j)] = 0.5f * (abs(vort[IX(i + 1, j)]) - abs(vort[IX(i - 1, j)]));
			gradVortY[IX(i, j)] = 0.5f * (abs(vort[IX(i, j + 1)]) - abs(vort[IX(i, j - 1)]));

			lenGrad[IX(i, j)] = sqrtf(gradVortX[IX(i, j)] * gradVortX[IX(i, j)] + gradVortY[IX(i, j)] * gradVortY[IX(i, j)]);
			if (lenGrad[IX(i, j)] < 0.01f)
			{
				vx0[IX(i, j)] = 0.0f;
				vy0[IX(i, j)] = 0.0f;
			}
			else
			{
				vx0[IX(i, j)] = gradVortX[IX(i, j)] / lenGrad[IX(i, j)];
				vy0[IX(i, j)] = gradVortY[IX(i, j)] / lenGrad[IX(i, j)];
			}
		}
	}
	set_bnd(vx0, 0);
	set_bnd(vy0, 0);

	for (int i = 1; i <= Nx; i++)
	{
		for (int j = 1; j <= Ny; j++)
		{
			vx[IX(i, j)] += vorticity * (vy0[IX(i, j)] * vort[IX(i, j)]);
			vy[IX(i, j)] += vorticity * (-vx0[IX(i, j)] * vort[IX(i, j)]);
		}
	}

	set_bnd(vx, 1);
	set_bnd(vy, 2);
}