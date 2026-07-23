#include "texnitis_nav_core/map_processing/map_preprocessor.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace nc = texnitis::nav_core;

TEST (MapPreprocessor, AcceptsPlain2DMap) {
    std::vector<int8_t>       cells (25, 0);
    nc::GridMapView           map{cells.data (), 5, 5, 0.1, 0.0, 0.0};
    nc::MapPreprocessorParams params;
    params.inflation_radius = 0.0;
    nc::ProcessedPlanningMapStorage storage;
    EXPECT_EQ (nc::MapPreprocessor (params).process (map, {}, storage), nc::PreprocessResult::Success);
    const auto output = storage.view ();
    ASSERT_NE (output.occupancy.data, nullptr);
    EXPECT_EQ (output.occupancy.at (2, 2), 0);
    EXPECT_FLOAT_EQ (output.traversal_cost[12], 0.0F);
}

TEST (MapPreprocessor, ElevationStepBecomesLethal) {
    std::vector<int8_t> cells (25, 0);
    std::vector<float>  elevation (25, 0.0F);
    elevation[12] = 0.20F;
    nc::GridMapView       map{cells.data (), 5, 5, 0.1, 0.0, 0.0};
    nc::TerrainLayersView terrain;
    terrain.elevation = nc::ElevationGridView{elevation.data (), 5, 5, 0.1, 0.0, 0.0};
    nc::MapPreprocessorParams params;
    params.inflation_radius = 0.0;
    params.max_step_height  = 0.05;
    nc::ProcessedPlanningMapStorage storage;
    ASSERT_EQ (nc::MapPreprocessor (params).process (map, terrain, storage), nc::PreprocessResult::Success);
    EXPECT_EQ (storage.view ().occupancy.at (2, 2), 100);
}

TEST (MapPreprocessor, RequiredTerrainMustExist) {
    std::vector<int8_t>       cells (4, 0);
    nc::GridMapView           map{cells.data (), 2, 2, 0.1, 0.0, 0.0};
    nc::MapPreprocessorParams params;
    params.missing_terrain_policy = nc::MissingTerrainPolicy::RequireElevation;
    nc::ProcessedPlanningMapStorage storage;
    EXPECT_EQ (nc::MapPreprocessor (params).process (map, {}, storage), nc::PreprocessResult::MissingTerrain);
}

TEST (MapPreprocessor, RejectsNullOccupancyData) {
    nc::ProcessedPlanningMapStorage storage;
    EXPECT_EQ (nc::MapPreprocessor ().process ({}, {}, storage), nc::PreprocessResult::InvalidMap);
}

TEST (MapPreprocessor, RejectsOccupiedThresholdOutsideDocumentedRange) {
    std::vector<int8_t>       cells (4, 0);
    nc::GridMapView           map{cells.data (), 2, 2, 0.1, 0.0, 0.0};
    nc::MapPreprocessorParams params;
    params.occupied_threshold = 101;
    nc::ProcessedPlanningMapStorage storage;
    EXPECT_EQ (nc::MapPreprocessor (params).process (map, {}, storage), nc::PreprocessResult::InvalidParameter);
}

TEST (MapPreprocessor, IgnoresNanTerrainCellWithoutMakingItLethal) {
    std::vector<int8_t>   cells (9, 0);
    std::vector<float>    elevation (9, NAN);
    nc::GridMapView       map{cells.data (), 3, 3, 0.1, 0.0, 0.0};
    nc::TerrainLayersView terrain;
    terrain.elevation = nc::ElevationGridView{elevation.data (), 3, 3, 0.1, 0.0, 0.0};
    nc::MapPreprocessorParams params;
    params.inflation_radius = 0.0;
    nc::ProcessedPlanningMapStorage storage;
    ASSERT_EQ (nc::MapPreprocessor (params).process (map, terrain, storage), nc::PreprocessResult::Success);
    EXPECT_EQ (storage.view ().occupancy.at (1, 1), 0);
}

TEST (MapPreprocessor, ValidMaskExcludesUnobservedElevation) {
    std::vector<int8_t>  cells (9, 0);
    std::vector<float>   elevation (9, 0.0F);
    std::vector<uint8_t> valid (9, 1U);
    elevation[4] = 1.0F;
    valid[4]     = 0U;
    nc::GridMapView       map{cells.data (), 3, 3, 0.1, 0.0, 0.0};
    nc::ElevationGridView elevation_view{elevation.data (), 3, 3, 0.1, 0.0, 0.0};
    elevation_view.valid_mask          = valid.data ();
    elevation_view.required_valid_bits = 1U;
    nc::TerrainLayersView terrain;
    terrain.elevation = elevation_view;
    nc::MapPreprocessorParams params;
    params.inflation_radius = 0.0;
    nc::ProcessedPlanningMapStorage storage;
    ASSERT_EQ (nc::MapPreprocessor (params).process (map, terrain, storage), nc::PreprocessResult::Success);
    EXPECT_EQ (storage.view ().occupancy.at (1, 1), 0);
}

TEST (TerrainGridView, RejectsInvalidStorageAndOutOfBoundsAccess) {
    nc::FloatGridView empty;
    EXPECT_FALSE (empty.valid ());
    EXPECT_TRUE (std::isnan (empty.at (0, 0)));

    std::vector<float> values (4, 1.0F);
    nc::FloatGridView grid{values.data (), 2, 2, 0.1, 0.0, 0.0};
    EXPECT_TRUE (std::isnan (grid.at (-1, 0)));
    EXPECT_TRUE (std::isnan (grid.at (2, 0)));
}

TEST (TerrainGridView, SlopeRequiresMatchingGridMetadata) {
    std::vector<float> values (4, 0.0F);
    nc::FloatGridView first{values.data (), 2, 2, 0.1, 0.0, 0.0};
    nc::FloatGridView matching{values.data (), 2, 2, 0.1, 0.0, 0.0};
    nc::FloatGridView mismatch{values.data (), 2, 1, 0.1, 0.0, 0.0};
    EXPECT_TRUE ((nc::SlopeGridView{first, matching}.valid ()));
    EXPECT_FALSE ((nc::SlopeGridView{first, mismatch}.valid ()));
}
