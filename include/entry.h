/**
 * Copyright (c) 2023 Grid-based Path Planning Competition and Contributors
 * <https://gppc.search-conference.org/>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef GPPC_ENTRY_H_
#define GPPC_ENTRY_H_

#include <string>
#include <vector>

// include common used class in GPPC
#include "gppc.h"

typedef gppc::xyLoc xyLoc;

/**
 * User code used during preprocessing of a map.
 * Can be left blank if no pre-processing is required.
 * It will not be called in the same program execution as the `PrepareForSearch` is called,
 * all data must be shared through files.
 *
 * Called with the command below:
 * ./run -pre <file.map>
 *
 * @param bits the data of the map, where (0,0) is the top-left corner.
 * bits.size() = height * width packed into 1D array, row-by-row,
 * i.e., the first `width` bool is the first row, y=0, and the next width is the second row, y=1
 * bits[i] returns `true` if (x,y) is traversable, `false` otherwise
 * @param width the width of the map
 * @param height the height of the map
 * @param filename The filename you write the preprocessing data to.  Open in `write` mode.
 */
void PreprocessMap(const std::vector<bool> &bits, int width, int height,
                   const std::string &filename);

/**
 * User code used to set up search before queries.
 * Can also load pre-processing data from files.
 * It will not be called in the same program execution as the `PreprocessMap` is called,
 * all data must be shared through files.
 *
 * Called with any commands below:
 * `./run -run <file.map> <file.map.scene>`
 * `./run -check <file.map> <file.map.scene>`
 *
 * @param bits the data of the map, where (0,0) is the top-left corner.
 * bits.size() = height * width packed into 1D array, row-by-row,
 * i.e., the first `width` bool is the first row, y=0, and the next width is the second row, y=1
 * bits[i] returns `true` if (x,y) is traversable, `false` otherwise
 * @param width the width of the map
 * @param height the height of the map
 * @param filename The filename you write the preprocessing data to. Open in `read` mode.
 * @returns Pointer to the data structure used for search. Memory should be stored on heap, not
 * stack.
 */
void *PrepareForSearch(const std::vector<bool> &bits, int width, int height,
                       const std::string &filename);

/**
 * User code used to run search queries.
 *
 * Called with any commands below:
 * `./run -run <file.map> <file.map.scene>`
 * `./run -check <file.map> <file.map.scene>`
 *
 * @param data Pointer to data returned from `PrepareForSearch`.
 * Can static_cast to correct data type.
 * @param s The start (x,y) coordinate of the search query
 * @param g The goal (x,y) coordinate of the search query
 * @param path The points that form the shortest path from `s` to `g` computed by the search
 * algorithm. The shortest path length will be calculated by summation of Euclidean distance between
 * consecutive pairs path[i]--path[i+1].
 * Collinear points are allowed. The path is empty if no shortest path exists.
 * @returns true if the pathfinding is completed (even if no path exists);
 * false if the pathfinding is not completed and requires further function calls,
 */
bool GetPath(void *data, xyLoc s, xyLoc g, std::vector<xyLoc> &path);

/**
 * @return the name of the algorithm
 */
std::string GetName();

#endif  // GPPC_ENTRY_H_
