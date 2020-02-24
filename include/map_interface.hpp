#pragma once
#include <cmath>
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <limits>
#include <array>
class MapInterface {

private:
  
    std::array<double,2>
        xrange_,    // Minimum and maximum range, respectivelly, on the x-axis
        yrange_;     // Minimum and maximum range, respectivelly, on the y-axis

    std::vector<float>
        map_;   // The array of values that store the map values by index
    std::size_t 
        size,   // Map size: must be nrow * ncol
        nrow,   // Number of rows of the map
        ncol;   // Number of cols of the map
    double
        min_val,    // The minimum value the map reaches 
        max_val,    // The maximum value the map reaches
        xdelta,     // Difference between the minimum and maximum x
        ydelta;     // Difference between the minimum and maximum y

    /* Given a string and a delimiter, split their characters into a vector of tokens.
     * Used by MapInterface::load().
     *
     * Parameters
     * -------------------------
     * str: the input string.
     * delim: the input delimiter.
     * cont: the output container. 
     */
    void splitString(const std::string& str, std::vector<std::string>& cont, char delim);

    /* Given a value on an given axis, return the index of the cell it is contained.
     * 
     * Parameters
     * --------------------------
     * v: value of the point in the coordinate system.
     * v_min: minimum value the point reaches.
     * v_max: maximum value the point reaches.
     * nb_cells: number of cells in which this coordinate system is divided.
     */
    static std::size_t getIdFromCoordinate(const double & v, const double & v_min, const double & v_max, const int & nb_cells);

public:

    /* Load the map from a ".map" file formatted as follows:
     * -> 1st line: 'nrow,ncol', where nrow,ncol are the number of rows and columns of the map grid
     * -> 2nd line: 'xmin,xmax', where xmin,xmax are the minimum and maximum values of the x coordinate range
     * -> 3rd line: 'ymin,ymax', where ymin,ymax are the minimum and maximum values of the y coordinate range
     * -> 4rd line: 'v0,v1,..., vi ,...,vN-1', where each 'v' is the value of the map grid at index i.
     * 
     * Parameters
     * --------------
     * path: the absolute location of the file.
     * 
     */
    void load(const std::string & path);

    /* Convert from cartesian coordinates to the map coordinates.
     *
     * Parameters
     * --------------
     * x, y: the coordinates of the map which are going to be
     *      converted to the grid position.
     * row, col: the output, the grid positions given x and y.
     * 
     */
    void toGridPosition(const double & x, const double & y, int & row, int & col) const;

    /* Retrieve the value of the map cell at row, col.
     *
     * Parameters
     * ---------------------
     * row,col: the row and col of the cell.
     */
    float at(const std::size_t & row, const std::size_t & col) const;

    /* Retrieve the value of the map at coordinates x and y
     *
     * Parameters
     * ---------------------
     * x,y: the coordinates of the point.
     */
    float at(const double & x, const double & y) const;

    /* Retrieve the range in the x coordinates
     * Parameters
     * --------------------
     * xrange: the reference to the array.
     */
    void getRangeX(std::array<double,2> & xrange) const;

    /* Retrieve the range in the y coordinates
     * Parameters
     * --------------------
     * yrange: the reference to the array.
     */
    void getRangeY(std::array<double,2> & yrange) const;

    /* Returns the number of rows of the map.
     */
    std::size_t getRows() const;

    /* Returns the number of columns of the map.
     */
    std::size_t getCols() const;

    /* Returns the minimum map value
     */
    float min() const;

    /* Returns the maximum map value
     */
    float max() const;

    /* Checks if coordinates are in map range
     * 
     * Parameters
     * ------------------------
     * x,y: the x and y coordinates
     */
    bool inRange(const double x, const double y) const;

};


void MapInterface::splitString(const std::string& str, std::vector<std::string>& cont, char delim) {
    cont.clear();
    std::stringstream ss(str);
    std::string token;
    while (std::getline(ss, token, delim)) {
        std::size_t idx = token.find(" ");
        while(idx != std::string::npos) {
            token.erase(idx,1);
            idx = token.find(" ");
        }
        if(!token.empty() && token.find(" ") == std::string::npos) {
            cont.push_back(token);
        }
    }
    return ;
}

void MapInterface::load(const std::string & path) {
    std::string temp;
    std::vector<std::string> tokens;
    std::ifstream map_file;
    map_file.open(path);
    if(!map_file.is_open()) {
        throw std::runtime_error("The map file could not be opened.");
    }

    // First line : load 'nrow,ncol'
    std::cout << "First line" << std::endl;
    std::getline(map_file,temp);
    MapInterface::splitString(temp, tokens, ',');
    this->nrow = (std::size_t) std::stoi(tokens[0]);
    this->ncol = (std::size_t) std::stoi(tokens[1]);
    this->size = this->nrow * this->ncol;
    
    // Second line: load 'xmin,xmax'
    std::cout << "Second line" << std::endl;
    std::getline(map_file,temp);
    MapInterface::splitString(temp, tokens, ',');
    this->xrange_[0] = std::stof(tokens[0]);
    this->xrange_[1] = std::stof(tokens[1]);
    this->xdelta = this->xrange_[1] - this->xrange_[0];

    // Third line: load 'ymin,ymax'
    std::cout << "third line" << std::endl;
    std::getline(map_file,temp);
    MapInterface::splitString(temp, tokens, ',');
    this->yrange_[0] = std::stof(tokens[0]);
    this->yrange_[1] = std::stof(tokens[1]);
    this->ydelta = this->yrange_[1] - this->yrange_[0];

    // Fourth line: load data (one line, csv)
    std::cout << "Fourth line" << std::endl;
    std::getline(map_file,temp);
    MapInterface::splitString(temp, tokens, ',');
    this->map_ = std::vector<float>(this->size);
    this->max_val = std::numeric_limits<float>::min();
    this->min_val = std::numeric_limits<float>::max();
    std::cout << tokens.size() << std::endl;
    for(std::size_t idx = 0; idx < this->size; idx++) {
	    float val = std::stof(tokens[idx]);
        this->max_val = (val > max_val) ? val : max_val;
        this->min_val = (val < min_val) ? val : min_val;
        this->map_[idx] = val;
    }
    
    std::cout << "Close"<< std::endl;
    // Finish input operation
    map_file.close();
    return ;
}

std::size_t MapInterface::getIdFromCoordinate(const double & v, const double & v_min, const double & v_max, const int & nb_cells) {
    double Delta_S = (v_max - v_min)/(1.0 * nb_cells);
    double i = ( (v - v_min)/(v_max-v_min) ) * (nb_cells - 1.0);
    std::size_t i_plus = std::ceil(i);
    std::size_t i_minus = std::floor(i);
    double c_plus = v_min + Delta_S * (i_plus + 0.5);
    double c_minus = v_min + Delta_S * (i_minus + 0.5);
    double d_plus = std::abs(v - c_plus);
    double d_minus = std::abs(v - c_minus);
    if(d_minus <= d_plus) {
        return i_minus;
    }
    return i_plus;
}

void MapInterface::toGridPosition(const double & x, const double & y, int & row, int & col) const {
    const double 
        xmin = this->xrange_[0],
        ymin = this->yrange_[0];
    col = getIdFromCoordinate(x,this->xrange_[0],this->xrange_[1],this->ncol); 
    row = getIdFromCoordinate(this->yrange_[1] - y + ymin, ymin,this->yrange_[1],this->nrow); // corrects, since rows go downwards
    return ;
}

float MapInterface::at(const std::size_t & row, const std::size_t & col) const {
    float val = this->map_[ row * this->ncol + col ];
    return val;
}

float MapInterface::at(const double & x, const double & y) const {
    int 
        row, 
        col;
    this->toGridPosition(x,y,row,col);
    return this->at((std::size_t) row, (std::size_t) col);
}

void MapInterface::getRangeX(std::array<double,2> & xrange) const {
    xrange[0] = this->xrange_[0];
    xrange[1] = this->xrange_[1];
    return ;
}

void MapInterface::getRangeY(std::array<double,2> & yrange) const {
    yrange[0] = this->yrange_[0];
    yrange[1] = this->yrange_[1];
    return ;
}

std::size_t MapInterface::getRows() const {
    return this->nrow;
}

std::size_t MapInterface::getCols() const {
    return this->ncol;
}

float MapInterface::min() const {
    return this->min_val;
}

float MapInterface::max() const {
    return this->max_val;
}

bool MapInterface::inRange(const double x, const double y) const {
    if ( 
        (this->xrange_[0] <= x && x <=this->xrange_[1]) && 
        (this->yrange_[0] <= y && y <=this->yrange_[1])
    ){
        return true;
    }
    return false;
}
