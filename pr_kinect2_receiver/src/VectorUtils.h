//                                     __
//    ____ ___  ___  ____ ___  ____   / /__   __
//   / __ `__ \/ _ \/ __ `__ \/ __ \ / __/ | / /
//  / / / / / /  __/ / / / / / /_/ // /_ | |/ /
// /_/ /_/ /_/\___/_/ /_/ /_/\____(_)__/ |___/
//
//
// Created by Memo Akten, www.memo.tv
//


#pragma once

#include <vector>
#include <sstream>
#include <random>

namespace msa {
    namespace vector_utils {
        
        // assuming typename Vector is a std::vector-like type with [] operators etc.
        
        //-------------------------------------------------------
        // convert to string
        template <typename Vector>
        std::string to_string(const Vector& v) {
            std::stringstream s;
            s << "[ ";
            for (const auto& f : v) s << f << " ";
            s << "]";
            return s.str();
        }
        
        
        
        //-------------------------------------------------------
        // return number of bytes
        template <typename Vector>
        int num_bytes(const Vector& v) {
            return sizeof(v[0]) * (int)v.size();
        }
        
        
        
        //-------------------------------------------------------
        // return min and max values of a 2d table (vector of vectors)
        template <typename Vector>
        void get_range(const std::vector< Vector >& data, Vector& min_values, Vector& max_values) {
            if (data.empty()) return;
            
            int row_size = (int)data[0].size();
            
            // start with first set of data for both min and max
            min_values =
            max_values = data[0];
            
            // iterate all entries in data
            for (const auto& row : data) {
                
                // find min&  max values for each element in row
                for (int i = 0; i < row_size; i++) {
                    min_values[i] = std::min( row[i], min_values[i] );
                    max_values[i] = std::max( row[i], max_values[i] );
                }
            }
        }
        
        
        //-------------------------------------------------------
        // return min and max values of a 2d table where storage is flattened
        template <typename Vector>
        void get_range(const Vector& data, int row_size, Vector& min_values, Vector& max_values) {
            if (data.empty()) return;
            
            // start with first set of data for both min and max
            min_values =
            max_values = Vector(data.begin(), data.begin() + row_size);
            
            // iterate all entries in data
            for(int row_start=0; row_start<data.size(); row_start+=row_size) {
                
                // find min&  max values for each element in row
                for (int i = 0; i < row_size; i++) {
                    min_values[i] = std::min( data[row_start + i], min_values[i] );
                    max_values[i] = std::max( data[row_start + i], max_values[i] );
                }
            }
        }
        
        
        
        //-------------------------------------------------------
        // simple linear map function, should put this elsewhere
        template <typename T>
        inline T map(T value, T input_min, T input_max, T output_min, T output_max) {
            if (input_max == input_min) return 0;
            return (value - input_min) / (input_max - input_min) * (output_max - output_min) + output_min;
        }
        
        
        
        //-------------------------------------------------------
        // normalize vector of values with range [min_values...max_values] (each of which is a vector), to the range [min_value...max_value]
        template <typename Vector, typename T>
        void normalize(const Vector& vector_in, const Vector& min_values, const Vector& max_values, const T min_value, const T max_value, Vector& norm_vector_out) {
            int n = (int)vector_in.size();
            int nr = min_values.size(); // range size: if vector_in.size() is greater than range size (i.e. for flattened data)
            
            // resize output vector if it's not the right size
            if(norm_vector_out.size() != n) norm_vector_out.resize(n);
            
            // iterate all elements and map to relevant range
            for (int i = 0; i < n; i++) norm_vector_out[i] = map(vector_in[i], min_values[i%nr], max_values[i%nr], min_value, max_value);
        }
        
        
        
        //-------------------------------------------------------
        // unnormalize vector of values with range [min_value...max_value], to the range [min_values...max_values] (each of which is a vector)
        template <typename Vector, typename T>
        void unnormalize(const Vector& norm_vector_in, const Vector& min_values, const Vector& max_values, const T min_value, const T max_value, Vector& vector_out) {
            int n = (int)norm_vector_in.size();
            int nr = min_values.size(); // range size: if vector_in.size() is greater than range size (i.e. for flattened data)
            
            // resize output vector if it's not the right size
            if(vector_out.size() != n) vector_out.resize(n);
            
            // iterate all elements and map to relevant range
            for (int i = 0; i < n; i++) vector_out[i] = map(norm_vector_in[i], min_value, max_value, min_values[i%nr], max_values[i%nr]);
        }
        
        
        
        //-------------------------------------------------------
        // unit tests for all of the above
        // data stored as vector of vectors
        template <typename DataVector, typename T>
        void test_2d(const int num_rows = 20, const int row_size = 15, const T min_rand = 3, const T max_rand = 9, const T norm_min = -1, const T norm_max = 1) {
            std::cout << std::endl << "msa::vector_utils::test_2d" << std::endl;
            
            std::vector< DataVector > data_orig(num_rows, DataVector(row_size));
            std::vector< DataVector > data_norm;    // normalized data
            std::vector< DataVector > data_unnorm;   // unnormalized data
            std::vector< DataVector > data_diff2;   // diff squared between original and unnormalized data
            DataVector min_values;
            DataVector max_values;
            
            // generate data
            {
                // init random generator
                std::default_random_engine rand_engine(std::random_device{}());
                std::uniform_real_distribution<T> rand_dist(min_rand, max_rand);
                
                // generate data
                std::cout << "generating data ... " << std::endl;
                for(auto& row : data_orig) for(auto& d : row) d = rand_dist(rand_engine);
                
                // calculate min and max values
                get_range(data_orig, min_values, max_values);
            }
            
            // original data
            {
                // calcualte sum and avg
                T sum(0), avg(0);
                for(const auto& row : data_orig) for(const auto& d : row) sum += d;
                avg = sum/(num_rows * row_size);
                
                // log
                for(const auto& row : data_orig) std::cout << to_string(row) << std::endl;
                std::cout << "min_values: " << to_string(min_values) << std::endl;
                std::cout << "max_values: " << to_string(max_values) << std::endl;
                std::cout << "Sum: " << sum << ", Avg: " << avg << std::endl;
            }
            
            // normalized data
            {
                std::cout << std::endl << "Normalized" << std::endl;
                data_norm = data_orig;   // to initialize sizes
                
                // normalize
                for(int i=0; i<num_rows; i++) normalize(data_orig[i], min_values, max_values, norm_min, norm_max, data_norm[i]);
                
                // calcualte sum and avg
                T sum(0), avg(0);
                for(const auto& row : data_norm) for(const auto& d : row) sum += d;
                avg = sum/(num_rows * row_size);
                
                // log
                for(const auto& row : data_norm) std::cout << to_string(row) << std::endl;
                std::cout << "Sum: " << sum << ", Avg: " << avg << std::endl;
            }
            
            // unnormalized data
            {
                std::cout << std::endl << "Un-Normalized" << std::endl;
                data_unnorm = data_norm; // to initialize sizes
                
                // unnormalize
                for(int i=0; i<num_rows; i++) unnormalize(data_norm[i], min_values, max_values, norm_min, norm_max, data_unnorm[i]);
                
                // calcualte sum and avg
                T sum(0), avg(0);
                for(const auto& row : data_unnorm) for(const auto& d : row) sum += d;
                avg = sum/(num_rows * row_size);
                
                // log
                for(const auto& row : data_unnorm) std::cout << to_string(row) << std::endl;
                std::cout << "Sum: " << sum << ", Avg: " << avg << std::endl;
            }
            
            // squared diff
            {
                std::cout << std::endl << "Squared diff" << std::endl;
                data_diff2 = data_orig; // to initialize sizes
                
                // unnormalize
                for(int i=0; i<num_rows; i++) for(int j=0; j<row_size; j++) { T e = data_orig[i][j] - data_unnorm[i][j]; data_diff2[i][j] = e * e; }
                
                // calcualte sum and avg
                T sum(0), avg(0);
                for(const auto& row : data_diff2) for(const auto& d : row) sum += d;
                avg = sum/(num_rows * row_size);
                
                // log
                for(const auto& row : data_diff2) std::cout << to_string(row) << std::endl;
                std::cout << "Sum: " << sum << ", Avg: " << avg << std::endl;
            }
            
            //
            
            
            std::cout << " /vector_utils::test_2d " << std::endl << std::endl << std::endl;
        }
        
        
        // data stored flat in single vector
        template <typename DataVector, typename T>
        void test_flat(const int num_rows = 20, const int row_size = 15, const T min_rand = 3, const T max_rand = 9, const T norm_min = -1, const T norm_max = 1) {
            std::cout << std::endl << "msa::vector_utils::test_flat" << std::endl;
            
            DataVector data_orig(num_rows * row_size);
            DataVector data_norm;    // normalized data
            DataVector data_unnorm;   // unnormalized data
            DataVector data_diff2;   // diff squared between original and unnormalized data
            DataVector min_values;
            DataVector max_values;
            
            // generate data
            {
                // init random generator
                std::default_random_engine rand_engine(std::random_device{}());
                std::uniform_real_distribution<T> rand_dist(min_rand, max_rand);
                
                // generate data
                std::cout << "generating data ... " << std::endl;
                for(auto& d : data_orig) d = rand_dist(rand_engine);
                
                // calculate min and max values
                get_range(data_orig, row_size, min_values, max_values);
            }
            
            // original data
            {
                // calcualte sum and avg
                T sum(0), avg(0);
                for(const auto& d : data_orig) sum += d;
                avg = sum/(num_rows * row_size);
                
                // log
                std::cout << to_string(data_orig) << std::endl;
                std::cout << "min_values: " << to_string(min_values) << std::endl;
                std::cout << "max_values: " << to_string(max_values) << std::endl;
                std::cout << "Sum: " << sum << ", Avg: " << avg << std::endl;
            }
            
            // normalized data
            {
                std::cout << std::endl << "Normalized" << std::endl;
                data_norm = data_orig;   // to initialize sizes
                
                // normalize
                normalize(data_orig, min_values, max_values, norm_min, norm_max, data_norm);
                
                // calcualte sum and avg
                T sum(0), avg(0);
                for(const auto& d : data_norm) sum += d;
                avg = sum/(num_rows * row_size);
                
                // log
                std::cout << to_string(data_norm) << std::endl;
                std::cout << "Sum: " << sum << ", Avg: " << avg << std::endl;
            }
            
            // unnormalized data
            {
                std::cout << std::endl << "Un-Normalized" << std::endl;
                data_unnorm = data_norm; // to initialize sizes
                
                // unnormalize
                unnormalize(data_norm, min_values, max_values, norm_min, norm_max, data_unnorm);
                
                // calcualte sum and avg
                T sum(0), avg(0);
                for(const auto& d : data_unnorm) sum += d;
                avg = sum/(num_rows * row_size);
                
                // log
                std::cout << to_string(data_unnorm) << std::endl;
                std::cout << "Sum: " << sum << ", Avg: " << avg << std::endl;
            }
            
            // squared diff
            {
                std::cout << std::endl << "Squared diff" << std::endl;
                data_diff2 = data_orig; // to initialize sizes
                
                // unnormalize
                for(int i=0; i<data_orig.size(); i++) { T e = data_orig[i] - data_unnorm[i]; data_diff2[i] = e * e; }
                
                // calcualte sum and avg
                T sum(0), avg(0);
                for(const auto& d : data_diff2) sum += d;
                avg = sum/(num_rows * row_size);
                
                // log
                std::cout << to_string(data_diff2) << std::endl;
                std::cout << "Sum: " << sum << ", Avg: " << avg << std::endl;
            }
            
            //
            
            
            std::cout << " /vector_utils::test_flat " << std::endl << std::endl << std::endl;
        }
        
        
        
    }
}
