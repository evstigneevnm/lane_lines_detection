/*
* This file is part of the Line Detection Demo Project v 0.01c distribution. 
(https://github.com/evstigneevnm/lane_lines_detection).
* Copyright (c) 2019 Dr. Evstigneev Nikolay Mikhaylovitch.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, version 2 only.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __GUARD_POLYNOMIAL_LSM_FIT_HPP__
#define __GUARD_POLYNOMIAL_LSM_FIT_HPP__

/*
    class for polynomial Mean Least Squares fitting.
    Only polynomials of degree 4 or less can be used
    Note that template parameter start with ONE, i.e. poly of order 1 is a constant etc...

    As an example, we use std::threads to accelerate Gramm matix construciton on a CPU
    ***
    TODO: - a better solution with constrains would be through Lagrange multipliers!
            Fix for later usage!

*/

#include <stdexcept>
#include <string>
#include <cmath>
#include <thread>

template<typename T, int poly_degree>
class polynomial_lms_fit
{
public:
    polynomial_lms_fit()
    {
        constrained_fit = false;
        common_constructor_operations();
    }

    polynomial_lms_fit(T x_point_for_given_derivative_, T value_of_derivative_): 
    x_point_for_given_derivative(x_point_for_given_derivative_), 
    value_of_derivative(value_of_derivative_)
    {
        constrained_fit = true;
        common_constructor_operations();
    }

 
    ~polynomial_lms_fit()
    {
        //i now that std::vector can't be deleted.
        //just to toy around with 0 size
        coefficients.erase(coefficients.begin());
    }



    void set_constrain(T x_point_for_given_derivative_, T value_of_derivative_)
    {   
        x_point_for_given_derivative = x_point_for_given_derivative_;
        value_of_derivative = value_of_derivative_;
        constrained_fit = true;
    }


    void fit(size_t size, const T*& x_array, const T*& y_array)
    {
        //TODO: the matrix is SPD, but we don't use this benifit here!
        
        // classical sequential fit
        zero_matrix();
        is_fitted=false;

        if(constrained_fit)
        {
            fit_arrays_constrained(size, x_array, y_array);
            solve_system(poly_degree-1);
            constrained_coefficients();

        }
        else
        {
            fit_arrays(size, x_array, y_array);
            solve_system(poly_degree);
        }

        is_fitted=true;

    }
  
    void fit(const std::vector<T>& x_array, const std::vector<T>& y_array)
    {

        //TODO: the matrix is SPD, but we don't use this benifit here!
        
        // c++11 parallel std::thread fit
        zero_matrix();
        is_fitted=false;

        if(constrained_fit)
        {
            fit_std_vectors_constrained(x_array, y_array);
            solve_system(poly_degree-1);
            constrained_coefficients();
        }
        else
        {
            fit_std_vectors(x_array, y_array);
            solve_system(poly_degree);
        }
           

        is_fitted=true;        
    }

    void fit(size_t size, const std::vector<T>& x_array, const std::vector<T>& y_array)
    {

        //TODO: the matrix is SPD, but we don't use this benifit here!
        
        // c++11 parallel std::thread fit
        zero_matrix();
        is_fitted=false;

        if(constrained_fit)
        {
            fit_std_vectors_constrained(size, x_array, y_array);
            solve_system(poly_degree-1);
            constrained_coefficients();
        }
        else
        {
            fit_std_vectors(size, x_array, y_array);
            solve_system(poly_degree);
        }
           

        is_fitted=true;        
    }

    
    T get_polynomial_value(const T x)
    {
        T ret=T(0);
        if(is_fitted)
        {
            for(int j=0;j<poly_degree;j++)
            {
                ret+=coefficients[j]*pow(x,j);
            }
        }
        else
        {
            throw std::runtime_error(std::string("Polynomial was not fitted"));
        }

        return ret;
    }


    void get_polynomial_value(const size_t size, const T*& x_array, T*& y_array)
    {
        for(size_t j=0;j<size;j++)
        {
            y_array[j]=get_polynomial_value(x_array[j]);
        }

    }


    void get_polynomial_value(const std::vector<T>& x_array, std::vector<T>& y_array)
    {
        size_t size = x_array.size();
        size_t size_y = y_array.size();
        if(size_y!=size)
            y_array.resize(size);

        for(size_t j=0;j<size;j++)
        {
            y_array[j]=get_polynomial_value(x_array[j]);
        }

    }


    void get_coefficients(std::vector<T>& coeff)
    {
        if(is_fitted)
            coeff = coefficients; //not too much to copy

    }


private:
    bool constrained_fit;
    bool is_fitted;
    std::vector<T> coefficients;
    T A[poly_degree][poly_degree+1];

    T x_point_for_given_derivative;
    T value_of_derivative;



    void common_constructor_operations()
    {
        if(poly_degree>5){
            throw std::runtime_error(std::string("Polynomial order > 4"));
        }
        is_fitted = false;
        coefficients.reserve(poly_degree); //resize
        for(unsigned int j=0;j<poly_degree;++j)
            coefficients.push_back(T(0));

        zero_matrix();

    }

    //here we assume that matrix is in the tridiagonal form.
    bool check_matrix_condition(int poly_degree_l)
    {
        T det=T(1);
        for(int j=0;j<poly_degree_l;j++)
            det*=A[j][j];

        if(std::abs<T>(det) < 1.0e-11)
            return false;
        else
            return true;

    }

    void solve_system(int poly_degree_l)
    {
        //just a stupid G-J elimination
        //TODO: change for SPD system matrix algo, i.e. Cholesky.
        //can use direct methods since matrix size is limited by 5 X 5

        for (int i=0;i<poly_degree_l;i++)
        {
            for (int k=i+1;k<poly_degree_l;k++)
            {
                if (std::abs<T>(A[i][i]) < std::abs<T>(A[k][i]))
                {
                    for (int j=0;j<=poly_degree_l;j++) 
                    {
                        T temp = A[i][j];
                        A[i][j] = A[k][j];
                        A[k][j] = temp;
                    }
                }
            }
        }
        for (int i=0;i<poly_degree_l-1;i++)
        {
            for (int k=i+1;k<poly_degree_l;k++)
            {
                T t=A[k][i]/A[i][i];
                for (int j=0;j<=poly_degree_l;j++)
                {
                    A[k][j] = A[k][j]-t*A[i][j];
                }
            }
        }

        if(check_matrix_condition(poly_degree_l))
        {
        
            for (int i=poly_degree_l-1;i>=0;i--)
            {                
                coefficients[i]=A[i][poly_degree_l];               
                for (int j=i+1;j<poly_degree_l;j++)
                {
                    if (j != i)
                    {
                        coefficients[i] = coefficients[i]-A[i][j]*coefficients[j];
                    }
                }
                coefficients[i] = coefficients[i]/A[i][i];
            }
        }
        else
        {
            //return previous coefficients and the message:
            std::cerr << "LMS Gramm matrix is close to singularity!" << std::endl;

        }
    }

    //struct for parallel execution
    struct Sum 
    {
        Sum()
        { 
            for(int j=0;j<poly_degree;j++)
                for(int k=0;k<=poly_degree;k++)
                    sum[j][k] = T(0); 
        }

        void operator ()(const std::vector<T>& x_, const std::vector<T>& y_, int begin, int end)
        {
            for(int j=begin; j<end; j++)
            {
                for(int k=0;k<poly_degree;k++)
                {
                    for(int l=0;l<poly_degree;l++)
                    {
                        sum[k][l] += pow(x_[j],(k+l)); 
                    }
                    sum[k][poly_degree] += y_[j]*pow(x_[j],k); 
                }
            }
     
        }
     
        T sum[poly_degree][poly_degree+1];

    };

    //we can either send *this of the containing class or pass parameters in constructor.
    //we choose to pass params, but it works in both ways!
    struct Sum_constrained 
    {
        Sum_constrained(T x0_, T q0_): x0(x0_), q0(q0_)
        { 
            for(int j=0;j<poly_degree;j++)
                for(int k=0;k<=poly_degree;k++)
                    sum[j][k] = T(0); 
        }

        void operator ()(const std::vector<T>& x_, const std::vector<T>& y_, int begin, int end)
        {
            for(int j=begin; j<end; j++)
            {
                for(int k=0;k<poly_degree-1;k++)
                {
                    for(int l=0;l<poly_degree-1;l++)
                    {
                        
                        sum[k][l] += C_j_k(x_[j], k)*C_j_k(x_[j], l);; 
                    }
                    sum[k][poly_degree-1] += C_j_k(x_[j], k)*(y_[j] - q0*x_[j]);
                }
            }
     
        }
     
        T sum[poly_degree][poly_degree+1];
    
    private:
        //TODO: check - inline works faster with g++ 4.8.5 ?!?
        inline T C_j_k(T x_j, int k)
        {
            int k_f = (k>0)?(k+1):(k);
            return pow(x_j,k_f) - T(k_f)*pow(x0,(k_f-1))*x_j;
        }
        T x0;
        T q0;

    };

    void zero_matrix()
    {
        for(int j=0;j<poly_degree;j++)
        {
            for (int k=0;k<=poly_degree;++k)
            {
                 A[j][k]=T(0);
            }
        } 
         
    }

    //just for a debug
    void print_matrix()
    {
        for(int j=0;j<poly_degree;j++)
        {
            for (int k = 0;  k <= poly_degree; ++ k)
            {
                printf("%lf ", A[j][k]);
            }
            printf("\n");
        }  

    }


    
    void fit_arrays(const size_t size, const T*& x_array, const T*& y_array)
    {
        for(size_t j=0;j<size;j++)
        {
            for(int k=0;k<poly_degree;k++)
            {
                for(int l=0;l<poly_degree;l++)
                {
                    A[k][l]+=pow(x_array[j],(k+l));
                }
                A[k][poly_degree]+=y_array[j]*pow(x_array[j],k);
            }
        }

    }
    

    inline T C_j_k(T x_j, int k)
    {
        int k_f = (k>0)?(k+1):(k);
        return pow(x_j,k_f) - T(k_f)*pow(x_point_for_given_derivative,(k_f-1))*x_j;
    }

    void fit_arrays_constrained(const size_t size, const T*& x_array, const T*& y_array)
    {

        for(size_t j=0;j<size;j++)
        {
            for(int k=0;k<poly_degree-1;k++)
            {
                for(int l=0;l<poly_degree-1;l++)
                {
                    T x = x_array[j];

                    A[k][l]+=C_j_k(x, k)*C_j_k(x, l);
                }
                A[k][poly_degree-1]+=C_j_k(x_array[j], k)*(y_array[j] - value_of_derivative*x_array[j]);
            }
        }

    }

    void fit_std_vectors(const std::vector<T>& x_array, const std::vector<T>& y_array)
    {
        size_t size = x_array.size();
        fit_std_vectors(size, x_array, y_array);
    }


    void fit_std_vectors(size_t size, const std::vector<T>& x_array, const std::vector<T>& y_array)
    {
        unsigned int n = std::thread::hardware_concurrency();
        
        std::vector<std::thread> threads;
        std::vector<Sum> Sums(n, Sum());

        std::vector<unsigned int> indecies(n+1, size/n);
        std::vector<unsigned int> sizes(n, size/n);


        for(unsigned int l=0;l<size%n;l++)
        {
            sizes[l]++;
        }
        unsigned int index_l = 0;
        for(unsigned int l=0;l<n;l++)
        {
            indecies[l] = index_l;
            index_l += sizes[l];
        }
        indecies[n]=index_l;


        for(unsigned int l=0;l<n;l++)
        {
            unsigned int begin_l = indecies[l];
            unsigned int end_l = indecies[l+1];

            //threads.push_back(std::thread([&Sums, &x_array, begin_l, end_l, l](){ Sums[l] = std::for_each(x_array.begin()+begin_l, x_array.begin()+end_l, Sums[l] );}));
            threads.push_back(std::thread(std::ref(Sums[l]), std::ref(x_array), std::ref(y_array), begin_l, end_l) );
            
        }
        
        //sum reduction
        for(unsigned int l=0;l<n;l++)
        {
            threads[l].join();
            for(int j=0;j<poly_degree;j++)
                for(int k=0;k<=poly_degree;k++)
                    A[j][k]+=Sums[l].sum[j][k];
        }

        
    }


    void fit_std_vectors_constrained(const std::vector<T>& x_array, const std::vector<T>& y_array)
    {
        size_t size = x_array.size();
        fit_std_vectors_constrained(size, x_array, y_array);
    }


    void fit_std_vectors_constrained(size_t size, const std::vector<T>& x_array, const std::vector<T>& y_array)
    {

        unsigned int n = std::thread::hardware_concurrency();
        
        std::vector<std::thread> threads;
        std::vector<Sum_constrained> Sums(n, Sum_constrained(x_point_for_given_derivative, value_of_derivative));

        std::vector<unsigned int> indecies(n+1, size/n);
        std::vector<unsigned int> sizes(n, size/n);


        for(unsigned int l=0;l<size%n;l++)
        {
            sizes[l]++;
        }
        unsigned int index_l = 0;
        for(unsigned int l=0;l<n;l++)
        {
            indecies[l] = index_l;
            index_l += sizes[l];
        }
        indecies[n]=index_l;


        for(unsigned int l=0;l<n;l++)
        {
            unsigned int begin_l = indecies[l];
            unsigned int end_l = indecies[l+1];

            threads.push_back(std::thread(std::ref(Sums[l]), std::ref(x_array), std::ref(y_array), begin_l, end_l) );
            
        }
        
        //sum reduction
        for(unsigned int l=0;l<n;l++)
        {
            threads[l].join();
            for(int j=0;j<poly_degree;j++)
                for(int k=0;k<=poly_degree;k++)
                    A[j][k]+=Sums[l].sum[j][k];
        }

        
    }


    void constrained_coefficients()
    {
        for(int k=1;k<poly_degree-1;k++)
            coefficients[k+1] = coefficients[k];

        T sum = T(0);
        for(int k=2;k<poly_degree;k++)
            sum+=T(k)*coefficients[k]*pow(x_point_for_given_derivative,k-1);

        coefficients[1] =  value_of_derivative-sum; 
        
    }



};




#endif