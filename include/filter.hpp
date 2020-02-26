#pragma once
#include <random>
#include <cfloat> // DBL_MAX
#include <cmath> // std::nextafter
#include "map_interface.hpp"

struct Particle {
public:
    double x, y, orientation;
    double w, delta_h, accum_h_stdev;

    int prev_row, prev_col;
    int curr_row, curr_col;

    bool flag_changed_cell;

    Particle() : 
        x(0),
        y(0), 
        orientation(0), 
        w(0), 
        delta_h(0), 
        accum_h_stdev(0),
        flag_changed_cell(false){ } ;

    Particle(
        const double x, const double y, 
        const float orientation, const float w
    ) : 
        x(x), y(y), 
        orientation(orientation), w(w),
        delta_h(0), accum_h_stdev(0),
        flag_changed_cell(false) { } ;
};

class RelativeAltitudeFilter {

protected:
    std::random_device  rand_dev_; // Used for random number generation.

    std::size_t nparticles_;
    std::vector<Particle> particles_;
    MapInterface map_;

    bool navigable_only_;
    std::vector< std::array<int, 2> > navigable_cells_; // Yes, I know... it would be more efficient to store the indices
    double nav_invalid_value_;

    static inline double gaussian(const double mean, const double stdev) {
        double exp_factor = - 0.5 * std::pow(mean/(stdev),2.0);
        double new_w = (1.0/(stdev * ( std::sqrt(2.0 * M_PI) ) )) * std::exp(exp_factor);
        return new_w;
    }

public:

    // Setup functions
    // ------------------------------------------------------------------------

    RelativeAltitudeFilter( const std::string & map_path ) : navigable_only_(false) { map_.load(map_path); }

    RelativeAltitudeFilter( const MapInterface & map ) : map_(map), navigable_only_(false) { }

    void setNavigableOnly ( const bool flag, const float nav_invalid_value = 0) { 
        navigable_only_ = flag;
        navigable_cells_.clear();
        nav_invalid_value_ = nav_invalid_value;
        if(navigable_only_ == true) {
            for(int row = 0; row < map_.getRows(); row++) {
                for(int col = 0; col < map_.getCols(); col++) {
                    if( map_.atRC(row,col) != nav_invalid_value){
                        std::array<int, 2> cell_pos = {row, col};
                        navigable_cells_.push_back(cell_pos);
                    }
                }
            }
        }
        return ;
    }

    const std::vector<Particle> & getParticles() const { return particles_; }

    void setParticles( const std::vector<Particle> & particles ) { 
        particles_ = particles; 
        nparticles_ = particles.size();
    } ;

    // Assumes orientation in [0, 2PI)
    void setOrientation( const float orientation, const float stdev ) {

        // Very low standard deviation
        if(stdev < 1e-16) {
            for (Particle & p : particles_) {
                p.orientation = orientation;
            }
            return;
        }

        // Usual standard deviation
        std::mt19937 generator(rand_dev_());
        std::normal_distribution<> dstr(0.0,stdev);
        for (Particle & p : particles_) {
            float dev_theta = dstr(generator);
            p.orientation = orientation + dev_theta;
            p.orientation = (p.orientation < 0) ? p.orientation + 2 * M_PI : p.orientation;
            p.orientation = (p.orientation >= 2 * M_PI) ? p.orientation - 2 * M_PI : p.orientation;
        }

        return ;
    }

    // ------------------------------------------------------------------------

    // Auxiliar functions
    // ------------------------------------------------------------------------

    void sampleUniform( const std::size_t nparticles ) {
        nparticles_ = nparticles;
        particles_ = std::vector<Particle>(nparticles_);

        std::array<double,2> xrange, yrange;
        map_.getRangeX(xrange);
        map_.getRangeY(yrange);
        const double weight = 1.0/(nparticles_);
        
        // Init random number generator
        std::mt19937        generator(rand_dev_());
        std::uniform_real_distribution<double>  distr_orientation(0.0, 1.98 * M_PI);
        
        // Sample points
        if(!navigable_only_) {
            std::uniform_real_distribution<double>  distr_x(xrange[0], xrange[1]);
            std::uniform_real_distribution<double>  distr_y(yrange[0], yrange[1]);
            for (Particle & p : particles_) {
                p.x = distr_x(generator);
                p.y = distr_y(generator);
                p.orientation = distr_orientation(generator);
                p.w = weight;
                map_.toGridPosition(p.x, p.y, p.curr_row, p.curr_col);
            }
        }
        else {
            std::uniform_int_distribution<std::size_t> dstr_ids(0, navigable_cells_.size()-1);
            for (Particle & p : particles_) {
                std::size_t idx = dstr_ids(generator);
                std::array<int, 2> cell_pos = navigable_cells_[idx];
                int row = cell_pos[0];
                int col = cell_pos[1];
                map_.fromGridPosition(row, col, p.x, p.y);
                p.orientation = distr_orientation(generator);
                p.w = weight;
                p.curr_row = row; p.curr_col = col;
            }
        }
        return ;
    }

    void print() const {
        for (uint idx = 0; idx < nparticles_; idx++) {
            const Particle & p = particles_[idx];
            std::cout << "[" << idx << "] " << 
                "(x=" << p.x << ", y=" << p.y << 
                ", yaw=" << p.orientation << 
                ", weight=" << p.w << ")" <<
                " - altitude: " << map_.atXY(p.x, p.y) <<
                " | delta_h: " << p.delta_h <<
                " | delta_h_stdev: " << p.accum_h_stdev << "| flag: " << p.flag_changed_cell << std::endl;
        }
    }

    // ------------------------------------------------------------------------

    // Filtering functions
    // ------------------------------------------------------------------------

    void predict (
        const double delta_x,
        const double delta_y,
        const float delta_orientation,
        const float stdev_x,
        const float stdev_y,
        const float stdev_orientation
    )  {
        std::mt19937 generator(rand_dev_());
        
        // Since there is a risk stdev_{x|y|theta} is zero, set a value to dstr_x,
        // but won't really matter in this case.
        float
            dstr_x = (stdev_x < 1e-10) ? (std::numeric_limits<double>::max()) : stdev_x,
            dstr_y = (stdev_y < 1e-10) ? (std::numeric_limits<double>::max()) : stdev_y,
            dstr_theta = (stdev_orientation < 1e-10) ? (std::numeric_limits<float>::max()) : stdev_orientation;
            
        std::normal_distribution<> 
            dstr_dev_x(0.0,dstr_x),
            dstr_dev_y(0.0,dstr_y),
            dstr_dev_theta(0.0,dstr_theta);

        // Offset previous position and orientation with the offset given
       for (Particle & p : particles_) {
            double dev_x = 0.0;
            double dev_y = 0.0;
            double dev_theta = 0.0;
            if(stdev_x != 0.0) { dev_x = dstr_dev_x(generator); }
            if(stdev_y != 0.0) { dev_y = dstr_dev_y(generator); }
            if(stdev_orientation != 0.0) { dev_theta = dstr_dev_theta(generator); }

            float orientation = p.orientation;

            int prev_row = p.curr_row;
            int prev_col = p.curr_col;

            // Rotate to the world coordinates: odom is a relation between the previous and the current frame
            // Therefore: P^t = R_{f^{t-1}}^W * delta + P^{t-1}
            p.x = p.x + cos(orientation) * (delta_x + dev_x) - sin(orientation) * (delta_y + dev_y);
            p.y = p.y + sin(orientation) * (delta_x + dev_x) + cos(orientation) * (delta_y + dev_y);
            int curr_row, curr_col;
            map_.toGridPosition(p.x, p.y, curr_row, curr_col);
            
            if ( ( prev_col != curr_col || prev_row != curr_row )) {
                p.prev_row = prev_row;
                p.prev_col = prev_col;
                p.curr_row = curr_row;
                p.curr_col = curr_col;
                p.flag_changed_cell = true;
            }

            if(navigable_only_ && map_.atRC( (std::size_t) curr_row, (std::size_t) curr_col ) == nav_invalid_value_) {
                p.w = 0;
                p.flag_changed_cell = true;
            }

            // Correct orientation to range [0,2PI)
            p.orientation = orientation + delta_orientation + dev_theta;
            if(p.orientation < 0) { p.orientation = p.orientation + 2.0 * M_PI; }
            if(p.orientation >= 2.0 * M_PI) { p.orientation = p.orientation - 2.0 * M_PI; }
       }

        // Resample particles out of bounds
        std::uniform_int_distribution<std::size_t> dstr_particles(0,nparticles_-1);
        // Are particles in map bounds?
        for (std::size_t idx = 0; idx < nparticles_; idx++) {
            std::size_t new_idx = idx;
            // Prevents infinite loops
            int counter = nparticles_ * 10;
            while(!map_.inRange(particles_[new_idx].x, particles_[new_idx].y) && counter >= 0) {
                new_idx = dstr_particles(generator);
                counter--;
            }
            if(counter < 0) {
                this->sampleUniform(nparticles_);
            }
            particles_[idx] = particles_[new_idx];
        }

        return ;
    }

    bool addAltitudeOffset(const double offset, const double stdev) {
        bool flag_updated_weights = false;
        for (Particle & p : particles_) {
            p.delta_h = p.delta_h + offset; 
            p.accum_h_stdev = std::sqrt(std::pow(p.accum_h_stdev,2) + std::pow(stdev,2)); // Sum of two Gaussian distributions, the resulting stdev is given like that.
            // Only updates if the particle moved on the grid
            if(p.flag_changed_cell) {
                double  
                    prev_h = map_.atRC( (std::size_t) p.prev_row, (std::size_t) p.prev_col),
                    curr_h = map_.atRC( (std::size_t) p.curr_row, (std::size_t) p.curr_col),
                    delta_h = curr_h - prev_h,
                    mean = delta_h - p.delta_h,
                    meas_stdev = p.accum_h_stdev;
                p.w = p.w * RelativeAltitudeFilter::gaussian(mean,meas_stdev);
                p.delta_h = 0;
                p.accum_h_stdev = 0;
                p.flag_changed_cell = false;
                flag_updated_weights = true;
            }
        }
        return flag_updated_weights;
    }

    void resample() {

        // Compute cummulative, normalized, weights
        double weight_sum = 0.0;
        for (std::size_t idx = 0; idx < nparticles_; idx++) {
            weight_sum += particles_[idx].w;
        }
        // If all the weights are zero, something must be wrong (all particles fall far from the true position)
        if (weight_sum < 1e-50) {
            this->sampleUniform(nparticles_);
            return ;
        }
        std::vector < double > cum_weights(nparticles_);
        double cum_sum = 0.0;
        for (std::size_t idx = 0; idx < nparticles_; idx++) {
            cum_sum += (particles_[idx].w / weight_sum);
            cum_weights[idx] = cum_sum;
        }

        // Sample particles by weight
        std::vector<Particle> new_particles(nparticles_);
        std::default_random_engine generator(rand_dev_());
        std::uniform_real_distribution<double> distribution(0.0,std::nextafter(1.0, DBL_MAX)); // Includes 1 to the range
        double w = 1./nparticles_;
        for (std::size_t old_idx = 0; old_idx < nparticles_; old_idx++) {
            double val = distribution(generator); // generates weight
            std::size_t new_idx = 0;
            double prev_w = 0.0;
            for (std::size_t particle_idx = 0; particle_idx < nparticles_;  particle_idx++) {
                if(particle_idx != 0) {
                    prev_w = cum_weights[particle_idx-1];
                }
                if(prev_w <= val && val <= cum_weights[particle_idx]) {
                    new_idx = particle_idx;
                    break;
                }
            }
            new_particles[old_idx] = particles_[new_idx];
            new_particles[old_idx].w = w;
        }
        
        particles_ = std::move(new_particles);
        return ;
    }

    // ------------------------------------------------------------------------
};
