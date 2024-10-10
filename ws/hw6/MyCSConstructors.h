#pragma once

// This includes all of the necessary header files in the toolbox
#include "AMPCore.h"

// Include the correct homework headers
#include "hw/HW4.h"
#include "hw/HW6.h"

////////////////////// THIS IS FROM HW4 //////////////////////

/* You can just move these classes to shared folder and include them instead of copying them to hw6 project*/

// Derive the amp::GridCSpace2D class and override the missing method
class MyGridCSpace2D : public amp::GridCSpace2D {
public:
    MyGridCSpace2D(std::size_t x0_cells, std::size_t x1_cells, double x0_min, double x0_max, double x1_min, double x1_max)
        : amp::GridCSpace2D(x0_cells, x1_cells, x0_min, x0_max, x1_min, x1_max),
          m_x0_min(x0_min), m_x0_max(x0_max), m_x1_min(x1_min), m_x1_max(x1_max),
          m_x0_cells(x0_cells), m_x1_cells(x1_cells) {}

    // Override this method for determining which cell a continuous point belongs to
    virtual std::pair<std::size_t, std::size_t> getCellFromPoint(double x0, double x1) const override;

    // Method to convert cell index to point coordinates in continuous space
    Eigen::Vector2d getPointFromCell(std::size_t i, std::size_t j) const;

    // Accessors for the number of cells in each dimension
    std::size_t getX0Cells() const { return m_x0_cells; }
    std::size_t getX1Cells() const { return m_x1_cells; }

private:
    double m_x0_min;
    double m_x0_max;
    double m_x1_min;
    double m_x1_max;
    std::size_t m_x0_cells;
    std::size_t m_x1_cells;
};





// Derive the HW4 ManipulatorCSConstructor class and override the missing method
class MyManipulatorCSConstructor : public amp::ManipulatorCSConstructor {
    public:
        // To make things easy, add the number of cells as a ctor param so you can easily play around with it
        MyManipulatorCSConstructor(std::size_t cells_per_dim) : m_cells_per_dim(cells_per_dim) {}

        // Override this method for computing all of the boolean collision values for each cell in the cspace
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::LinkManipulator2D& manipulator, const amp::Environment2D& env) override;

        bool isPointInPolygon(const Eigen::Vector2d& point, const amp::Polygon& polygon) const;
        bool doLinesIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& q1, const Eigen::Vector2d& p2, const Eigen::Vector2d& q2) const;
        bool doLinesIntersect(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, 
                              const amp::Polygon& polygon) const;

    private:
        std::size_t m_cells_per_dim;
};

//////////////////////////////////////////////////////////////

// Derive the PointAgentCSConstructor class and override the missing method
class MyPointAgentCSConstructor : public amp::PointAgentCSConstructor {
    public:
        // To make things easy, add the number of cells as a ctor param so you can easily play around with it
        MyPointAgentCSConstructor(std::size_t cells_per_dim) : m_cells_per_dim(cells_per_dim) {}

        // Override this method for computing all of the boolean collision values for each cell in the cspace
        virtual std::unique_ptr<amp::GridCSpace2D> construct(const amp::Environment2D& env) override;

    private:
        std::size_t m_cells_per_dim;
        bool isPointInPolygon(const Eigen::Vector2d& point, const amp::Polygon& polygon) const;
};

class MyWaveFrontAlgorithm : public amp::WaveFrontAlgorithm {
    public:
        virtual amp::Path2D planInCSpace(const Eigen::Vector2d& q_init, const Eigen::Vector2d& q_goal, const amp::GridCSpace2D& grid_cspace, bool isManipulator) override;

};

