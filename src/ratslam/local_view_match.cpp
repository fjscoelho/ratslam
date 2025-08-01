/*
 * openRatSLAM
 *
 * utils - General purpose utility helper functions mainly for angles and readings settings
 *
 * Copyright (C) 2012
 * David Ball (david.ball@qut.edu.au) (1), Scott Heath (scott.heath@uqconnect.edu.au) (2)
 *
 * RatSLAM algorithm by:
 * Michael Milford (1) and Gordon Wyeth (1) ([michael.milford, gordon.wyeth]@qut.edu.au)
 *
 * 1. Queensland University of Technology, Australia
 * 2. The University of Queensland, Australia
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// This file implements the LocalViewMatch class for RatSLAM.
// All code is written to be compatible with ROS 2 and follows ROS 2 C++ style guidelines.
// For code style, see ament_uncrustify, ament_cpplint, and ament_clang_format.

#include "local_view_match.h"
#include "Image.hh"
#include "VisualTemplate.hh"

#include <assert.h>
#include <stdlib.h>
#include <string.h>

#include <iomanip>
#include <iostream>

#include "utils.h"
using namespace std;
#include <stdio.h>

#include <algorithm>
#include <boost/foreach.hpp>

namespace ratslam
{

  LocalViewMatch::LocalViewMatch(
    double vt_min_patch_normalisation_std, int vt_patch_normalisation, double vt_normalisation,
    int vt_shift_match, int vt_step_match, int vt_panoramic, double vt_match_threshold,
    bool vt_threshold_condition, int template_x_size, int template_y_size, int image_crop_x_min,
    int image_crop_x_max, int image_crop_y_min, int image_crop_y_max)
  {
    VT_MIN_PATCH_NORMALISATION_STD = vt_min_patch_normalisation_std;
    VT_PATCH_NORMALISATION = vt_patch_normalisation;
    VT_NORMALISATION = vt_normalisation;
    VT_SHIFT_MATCH = vt_shift_match;
    VT_STEP_MATCH = vt_step_match;
    VT_PANORAMIC = vt_panoramic;
    VT_MATCH_THRESHOLD = vt_match_threshold;
    VT_THRESHOLD_CONDITION = vt_threshold_condition;
    TEMPLATE_X_SIZE = template_x_size;
    TEMPLATE_Y_SIZE = template_y_size;
    IMAGE_VT_X_RANGE_MIN = image_crop_x_min;
    IMAGE_VT_X_RANGE_MAX = image_crop_x_max;
    IMAGE_VT_Y_RANGE_MIN = image_crop_y_min;
    IMAGE_VT_Y_RANGE_MAX = image_crop_y_max;

    TEMPLATE_SIZE = TEMPLATE_X_SIZE * TEMPLATE_Y_SIZE;
    templates.reserve(10000);
    current_view.resize(TEMPLATE_SIZE);
    current_vt = 0;
    prev_vt = 0;

    _vtRelativeRad = 0;
    _maxVtId     = -1;
    _activeVtId  =  0;
    _prevVtId    =  0;
    _lastSavedId = -1;  // No templated was saved, yet.
  }

  LocalViewMatch::~LocalViewMatch() {}

  void LocalViewMatch::on_image(
    const unsigned char * view_rgb, bool greyscale, unsigned int image_width,
    unsigned int image_height)
  {
    int _templateDimX = TEMPLATE_X_SIZE;
    int _templateDimY = TEMPLATE_Y_SIZE;
    int _imageVtMinX = IMAGE_VT_X_RANGE_MIN;
    int _imageVtMaxX =  IMAGE_VT_X_RANGE_MAX;
    int _imageVtMinY =  IMAGE_VT_Y_RANGE_MIN;
    int _imageVtMaxY =  IMAGE_VT_Y_RANGE_MAX;
    double _vtNormFactor = VT_NORMALISATION; 

    // Create ratslam::Image from raw data
    ratslam::Image *image = new ratslam::Image(0, greyscale, image_width, image_height);
    size_t num_channels = greyscale ? 1 : 3;
    size_t img_size = image_width * image_height * num_channels;
    unsigned char *data_copy = new unsigned char[img_size];
    memcpy(data_copy, view_rgb, img_size);
    image->setData(data_copy);

    VisualTemplate *newVt =
      VisualTemplate::create(_vtSet.size(), image,
                            _imageVtMinX, _imageVtMaxX,
                            _imageVtMinY, _imageVtMaxY,
                            _templateDimX, _templateDimY,
                            _vtNormFactor
                            );
    delete image->data;

    // Compare new template with all stored templates.
    int    vtMatchId;
    double vtError = DBL_MAX;
    compare( newVt, vtError, vtMatchId );

    if ( vtError <= VT_MATCH_THRESHOLD )
    {
      // If 'newVt' matchs, activate the matched VT.
      if ( _activeVtId != vtMatchId )
        _prevVtId = _activeVtId;
      
      _activeVtId = vtMatchId;
      delete newVt;
    }
    else
    {
      // If there is no match, insert new created VT into '_vtSet'.
      _vtRelativeRad = 0;

      _vtSet.add( newVt );
      
      _maxVtId = newVt->getId();
      
      _prevVtId   = _activeVtId;
      _activeVtId = newVt->getId();
    }

    // Resulting values.
    //*vtId   = _activeVtId;
    //*relRad = _vtRelativeRad;  
  }

  /***************/
  /*** compare ***/
  void
  LocalViewMatch::compare( VisualTemplate *newVt, double &vtError,
                      int &vtMatchId )
  {
    if ( _vtSet.empty() )
    {
      vtError = DBL_MAX;
      return;
    }

    // Compatibility with old implementations.
    if ( VT_PANORAMIC )
    {
      cerr <<"RecodedLV::compare() - comparePanoramic() not implemented."
          << endl;
      return;
    }

    int vtShift = VT_SHIFT_MATCH;
    int vtMatch = VT_STEP_MATCH;
    compareStandard ( newVt, vtShift, vtMatch, vtError, vtMatchId );
  }


  /************************/
  /*** compare Standard ***/
  void
  LocalViewMatch::compareStandard( VisualTemplate *newVt,
                              int shiftMatch, int stepMatch,
                              double &vtError, int &vtMatchId )
  {
    double epsilon   = 0.005;
    double threshold = VT_MATCH_THRESHOLD + epsilon;
    double newMean   = newVt->mean();

    // Stores current mininum difference.
    double minDiff = vtError = DBL_MAX;

    // Run over all stored VT, in reverse order.
    //
    VisualTemplateSet::iterator it = _vtSet.begin();
    while ( it != _vtSet.end() )
    {
      VisualTemplate *vt = *it++;

      if ( ::abs( newMean - vt->mean() ) > threshold )
        continue;

      // Note: 'minDiff' is internally used by 'VisualTemplate::compare()'.
      if ( newVt->compare( vt, shiftMatch, stepMatch, &minDiff, &vtError ) )
        vtMatchId = vt->getId();
    }

    _vtRelativeRad = 0;
  }

}  // namespace ratslam
