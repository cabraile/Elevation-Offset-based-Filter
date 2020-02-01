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
class MapInterface {

private:

    float 
        *map;   // The array of values that store the map values by index
    std::size_t 
        size,   // Map size: must be nrow * ncol
        nrow,   // Number of rows of the map
        ncol;   // Number of cols of the map
    float
        min_val,    // The minimum value the map reaches 
        max_val,    // The maximum value the map reaches
        xdelta,     // Difference between the minimum and maximum x
        ydelta,     // Difference between the minimum and maximum y
        xrange[2],  // Minimum and maximum range, respectivelly, on the x-axis
        yrange[2];  // Minimum and maximum range, respectivelly, on the y-axis

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
    static std::size_t getIdFromCoordinate(const float & v, const float & v_min, const float & v_max, const int & nb_cells);

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
    void toGridPosition(const float & x, const float & y, int & row, int & col) const;

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
    float at(const float & x, const float & y) const;

    float * getRangeX() const;

    float * getRangeY() const;

    std::size_t getRows() const;

    std::size_t getCols() const;

    float min() const;

    float max() const;

    bool inRange(const float x, const float y) const;

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

    // First line : load 'nrow,ncol'
    std::getline(map_file,temp);
    MapInterface::splitString(temp, tokens, ',');
    this->nrow = (std::size_t) std::stoi(tokens[0]);
    this->ncol = (std::size_t) std::stoi(tokens[1]);
    this->size = this->nrow * this->ncol;
    
    // Second line: load 'xmin,xmax'
    std::getline(map_file,temp);
    MapInterface::splitString(temp, tokens, ',');
    this->xrange[0] = std::stof(tokens[0]);
    this->xrange[1] = std::stof(tokens[1]);
    this->xdelta = this->xrange[1] - this->xrange[0];

    // Third line: load 'ymin,ymax'
    std::getline(map_file,temp);
    MapInterface::splitString(temp, tokens, ',');
    this->yrange[0] = std::stof(tokens[0]);
    this->yrange[1] = std::stof(tokens[1]);
    this->ydelta = this->yrange[1] - this->yrange[0];

    // Fourth line: load data (one line, csv)
    std::getline(map_file,temp);
    MapInterface::splitString(temp, tokens, ',');
    this->map = (float *) malloc(  this->size * sizeof(float) );
    this->max_val = std::numeric_limits<float>::min();
    this->min_val = std::numeric_limits<float>::max();
    for(std::size_t idx = 0; idx < this->size; idx++) {
	    float val = std::stof(tokens[idx]);
        this->max_val = (val > max_val) ? val : max_val;
        this->min_val = (val < min_val) ? val : min_val;
        this->map[idx] = val;
    }

    // Finish input operation
    map_file.close();
    return ;
}

std::size_t MapInterface::getIdFromCoordinate(const float & v, const float & v_min, const float & v_max, const int & nb_cells) {
    float Delta_S = (v_max - v_min)/(1.0 * nb_cells);
    float i = ( (v - v_min)/(v_max-v_min) ) * (nb_cells - 1.0);
    std::size_t i_plus = std::ceil(i);
    std::size_t i_minus = std::floor(i);
    float c_plus = v_min + Delta_S * (i_plus + 0.5);
    float c_minus = v_min + Delta_S * (i_minus + 0.5);
    float d_plus = std::abs(v - c_plus);
    float d_minus = std::abs(v - c_minus);
    if(d_minus <= d_plus) {
        return i_minus;
    }
    return i_plus;
}

void MapInterface::toGridPosition(const float & x, const float & y, int & row, int & col) const {
    const float 
        xmin = this->xrange[0],
        ymin = this->yrange[0];
    col = getIdFromCoordinate(x,this->xrange[0],this->xrange[1],this->ncol); 
    //row = getIdFromCoordinate(y,this->yrange[0],this->yrange[1],this->nrow); 
    row = getIdFromCoordinate(this->yrange[1] - y + ymin, ymin,this->yrange[1],this->nrow); // corrects, since rows go downwards
    return ;
}

float MapInterface::at(const std::size_t & row, const std::size_t & col) const {
    float val = this->map[ row * this->ncol + col ];
    return val;
}

float MapInterface::at(const float & x, const float & y) const {
    int 
        row, 
        col;
    this->toGridPosition(x,y,row,col);
    return this->at((std::size_t) row, (std::size_t) col);
}

float * MapInterface::getRangeX() const {
    float * xrange = (float *) malloc (2 * sizeof(float));
    xrange[0] = this->xrange[0];
    xrange[1] = this->xrange[1];
    return xrange;
}

float * MapInterface::getRangeY() const {
    float * yrange = (float *) malloc (2 * sizeof(float));
    yrange[0] = this->yrange[0];
    yrange[1] = this->yrange[1];
    return yrange;
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

bool MapInterface::inRange(const float x, const float y) const {
    if ( 
        (this->xrange[0] <= x && x <=this->xrange[1]) && 
        (this->yrange[0] <= y && y <=this->yrange[1])
    ){
        return true;
    }
    return false;
}
