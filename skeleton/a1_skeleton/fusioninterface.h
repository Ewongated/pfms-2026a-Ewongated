#ifndef FUSIONINTERFACE_H
#define FUSIONINTERFACE_H

#include <vector>
#include "rangerinterface.h"
#include "cell.h"

/**
 * @brief Specifies the required interface for your Fusion class your ranger fusion
// class must inherit from it. <b> You MUST NOT edit this file </b>.
 * 
 */

/*!
 *  \brief     RangerFusiom Interface Class
 *  \details
 *  Specifies the required interface for your Fusion class your ranger fusion
 * class must inherit from it. <b> You MUST NOT edit this file </b>.
 *  \author    Alen Alempijevic
 *  \version   1.00-1
 *  \date      2020-04-10
 *  \pre       none
 *  \bug       none reported as of 2020-04-11
 *  \warning   students MUST NOT change this class (the header file)
 */


class FusionInterface
{
public:
    FusionInterface(){};

    /**
     * @brief Accepts the container of cells.
     *
     * @param cells
     */
    virtual void setCells(std::vector<pfms::Cell*> cells) = 0;

    /**
     * @brief Does two operations (1) Calls each ranger to generate data and uses this data to determine colissions
     * with provided container of cells (2) Generates a 'fusion' of the data based on collision conditions as
     * descibed in Assignment 2 specification
     *
     */
    virtual void grabAndFuseData() = 0;

    /**
     * @brief Returns the raw data from all sensors in the ranger container within a vector of vectors
     * The raw data is updated every time a new fusion is requested (grabAndFuseDat). The raw data is the data
     * used for collision checking. If no fusion has occured the vector shall be empty.
     *
     * @return std::vector<std::vector<double>>  the outer elements of the vector related to the rangers, the inner elements of vector are the respective range readings

     *
     * @sa grabAndFuseData
     */
    virtual std::vector<std::vector<double>> getRawRangeData() = 0;

    /**
     * @brief Returns the centre of all visible cube and cylinder objects as a vector of doubles.
     * Uses the PONIT sensor to achieve this. The data is updated every time a new fusion is requested (grabAndFuseDat). 
     * 
     * @return std::vector<double> Centre of all visible cube and cylinder objects as a vector of doubles. The first three elements of the vector are the x, y, z coordinates of the centre of the object. If no fusion has occured the vector shall be empty.
     *
     * @sa grabAndFuseData
     */
    virtual std::vector<double> getObjectDentre() = 0;


};

#endif // FUSIONINTERFACE_H
