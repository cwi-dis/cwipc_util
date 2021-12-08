// cwi_encode.cpp : Defines the exported functions for the DLL application.
//
#include <cstdint>
#include <chrono>
#include <sstream>
#ifdef WIN32
#define _CWIPC_UTIL_EXPORT __declspec(dllexport)
#else
#define _CWIPC_UTIL_EXPORT
#endif

#define PCL_NO_PRECOMPILE

#include "cwipc_util/api_pcl.h"
#include "cwipc_util/api.h"

#include <pcl/point_cloud.h>
#include <pcl/exceptions.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

namespace pcl {
template class VoxelGrid<cwipc_pcl_point>;
}

cwipc *cwipc_downsample(cwipc *pc, float voxelsize)
{
	if (pc == NULL) return NULL;
	cwipc_pcl_pointcloud src = pc->access_pcl_pointcloud();
	if (src == NULL) return NULL;
	float oldvoxelsize = pc->cellsize();
	if (oldvoxelsize >= voxelsize) voxelsize = oldvoxelsize;
	cwipc_pcl_pointcloud dst = new_cwipc_pcl_pointcloud();
	// Step 1 - Voxelize
	try {
		pcl::VoxelGrid<cwipc_pcl_point> grid;
		grid.setInputCloud(src);
		grid.setLeafSize(voxelsize, voxelsize, voxelsize);
		grid.setSaveLeafLayout(true);
		grid.filter(*dst);
		// Step 2 - Clear tile numbers in destination
		for (auto dstpt : dst->points) {
			dstpt.a = 0;
		}
		// Step 3 - Do OR of all contribution point tile numbers in destination.
		for (auto srcpt : src->points) {
			auto dstIndex = grid.getCentroidIndex(srcpt);
			auto dstpt = dst->points[dstIndex];
			dstpt.a |= srcpt.a;
		}
	} catch (pcl::PCLException& e) {
		std::cerr << "cwipc_downsample: PCL exception: " << e.detailedMessage() << std::endl;
		return NULL;
	}
	catch (std::exception& e) {
		std::cerr << "cwipc_downsample: std exception: " << e.what() << std::endl;
		return NULL;	
	}
	cwipc *rv = cwipc_from_pcl(dst, pc->timestamp(), NULL, CWIPC_API_VERSION);
	rv->_set_cellsize(voxelsize);

	// copy src auxdata to dst pointcloud
	cwipc_auxiliary_data* src_ad = pc->access_auxiliary_data();
	cwipc_auxiliary_data* dst_ad = rv->access_auxiliary_data();
	src_ad->_move(dst_ad);

	return rv;
}

/// <summary>
/// All points who have a distance larger than StddevMulThresh standard deviation of the mean distance to the query point will be marked as outliers and removed
/// </summary>
/// <param name="pc">Source point cloud to be cleaned</param>
/// <param name="kNeighbors">number of neighbors to analyze for each point</param>
/// <param name="stddevMulThresh">standard deviation multiplier</param>
/// <returns>Cleaned point cloud</returns>
cwipc_pcl_pointcloud cwipc_remove_outliers(cwipc* pc, int kNeighbors, float stddevMulThresh)
{
	if (pc == NULL) return NULL;
	cwipc_pcl_pointcloud src = pc->access_pcl_pointcloud();
	if (src == NULL) return NULL;
	cwipc_pcl_pointcloud dst = new_cwipc_pcl_pointcloud();

	// Apply statistical outlier removal 
	try {
		pcl::StatisticalOutlierRemoval<cwipc_pcl_point> sor;
		sor.setInputCloud(src);
		sor.setMeanK(kNeighbors);
		sor.setStddevMulThresh(stddevMulThresh);
		sor.filter(*dst);
	}
	catch (pcl::PCLException& e) {
		std::cerr << "cwipc_downsample: PCL exception: " << e.detailedMessage() << std::endl;
		return NULL;
	}
	catch (std::exception& e) {
		std::cerr << "cwipc_downsample: std exception: " << e.what() << std::endl;
		return NULL;
	}
	return dst;
}


/// <summary>
/// All points who have a distance larger than StddevMulThresh standard deviation of the mean distance to the query point will be marked as outliers and removed
/// </summary>
/// <param name="pc">Source point cloud to be cleaned</param>
/// <param name="kNeighbors">number of neighbors to analyze for each point</param>
/// <param name="stddevMulThresh">standard deviation multiplier</param>
/// <param name="perTile">decides if apply the filter per tile or to the full pointcloud</param>
/// <returns>Cleaned point cloud</returns>
cwipc* cwipc_remove_outliers(cwipc* pc, int kNeighbors, float stddevMulThresh, bool perTile)
{
	if (pc == NULL) return NULL;
	cwipc_pcl_pointcloud src = pc->access_pcl_pointcloud();
	if (src == NULL) return NULL;
	cwipc_pcl_pointcloud dst = new_cwipc_pcl_pointcloud();

	// Apply statistical outlier removal 
	try {
		if (perTile) {
			std::cout << "cwipc_util: cwipc_remove_outliers: Removing outliers per tile" << std::endl;
			std::vector< int > tiles;
			for (auto pt : src->points) {
				int tile = pt.a;
				if (std::find(tiles.begin(), tiles.end(), tile) != tiles.end()) {
					//std::cout << "Element found";
					continue;
				}
				else {
					tiles.push_back(tile);
				}
			}
			//std::cout << "Found " << tiles.size() << " tiles " << std::endl;
			for (int tile : tiles)
			{
				cwipc* aux_pc = cwipc_tilefilter(pc, tile);
				cwipc_pcl_pointcloud aux_dst = cwipc_remove_outliers(aux_pc, kNeighbors, stddevMulThresh);
				*dst += *aux_dst;
				aux_pc->free();
				//std::cout << "Cleaned tile " << tile << std::endl;
			}
			cwipc* rv = cwipc_from_pcl(dst, pc->timestamp(), NULL, CWIPC_API_VERSION);

			// copy src auxdata to dst pointcloud
			cwipc_auxiliary_data* src_ad = pc->access_auxiliary_data();
			cwipc_auxiliary_data* dst_ad = rv->access_auxiliary_data();
			src_ad->_move(dst_ad);

			return rv;
		}
		else {
			std::cout << "Removing outliers on the full pointcloud" << std::endl;
			dst = cwipc_remove_outliers(pc, kNeighbors, stddevMulThresh);
			cwipc* rv = cwipc_from_pcl(dst, pc->timestamp(), NULL, CWIPC_API_VERSION);

			// copy src auxdata to dst pointcloud
			cwipc_auxiliary_data* src_ad = pc->access_auxiliary_data();
			cwipc_auxiliary_data* dst_ad = rv->access_auxiliary_data();
			src_ad->_move(dst_ad);

			return rv;
		}
	}
	catch (pcl::PCLException& e) {
		std::cerr << "cwipc_downsample: PCL exception: " << e.detailedMessage() << std::endl;
		return NULL;
	}
	catch (std::exception& e) {
		std::cerr << "cwipc_downsample: std exception: " << e.what() << std::endl;
		return NULL;
	}
	return pc;
}


cwipc *cwipc_tilefilter(cwipc *pc, int tile)
{
    if (pc == NULL) return NULL;
    cwipc_pcl_pointcloud src = pc->access_pcl_pointcloud();
    if (src == NULL) return NULL;
    cwipc_pcl_pointcloud dst = new_cwipc_pcl_pointcloud();
    for (auto pt : src->points) {
        if (tile == 0 || tile == pt.a) {
            dst->points.push_back(pt);
        }
    }
    cwipc *rv = cwipc_from_pcl(dst, pc->timestamp(), NULL, CWIPC_API_VERSION);
    rv->_set_cellsize(pc->cellsize());

	// copy src auxdata to dst pointcloud
	cwipc_auxiliary_data* src_ad = pc->access_auxiliary_data();
	cwipc_auxiliary_data* dst_ad = rv->access_auxiliary_data();
	src_ad->_move(dst_ad);

    return rv;
}

cwipc *cwipc_tilemap(cwipc *pc, uint8_t map[256])
{
    if (pc == NULL) return NULL;
    cwipc_pcl_pointcloud src = pc->access_pcl_pointcloud();
    if (src == NULL) return NULL;
    cwipc_pcl_pointcloud dst = new_cwipc_pcl_pointcloud();
    for (auto pt : src->points) {
        pt.a = map[pt.a];
        dst->points.push_back(pt);
    }
    cwipc *rv = cwipc_from_pcl(dst, pc->timestamp(), NULL, CWIPC_API_VERSION);
    rv->_set_cellsize(pc->cellsize());

	// copy src auxdata to dst pointcloud
	cwipc_auxiliary_data* src_ad = pc->access_auxiliary_data();
	cwipc_auxiliary_data* dst_ad = rv->access_auxiliary_data();
	src_ad->_move(dst_ad);

    return rv;
}

cwipc *cwipc_colormap(cwipc *pc, uint32_t clearBits, uint32_t setBits)
{
    if (pc == NULL) return NULL;
    cwipc_pcl_pointcloud src = pc->access_pcl_pointcloud();
    if (src == NULL) return NULL;
    cwipc_pcl_pointcloud dst = new_cwipc_pcl_pointcloud();
    for (auto pt : src->points) {
        pt.rgba &= ~clearBits;
        pt.rgba |= setBits;
        dst->points.push_back(pt);
    }
    cwipc *rv = cwipc_from_pcl(dst, pc->timestamp(), NULL, CWIPC_API_VERSION);
    rv->_set_cellsize(pc->cellsize());

	// copy src auxdata to dst pointcloud
	cwipc_auxiliary_data* src_ad = pc->access_auxiliary_data();
	cwipc_auxiliary_data* dst_ad = rv->access_auxiliary_data();
	src_ad->_move(dst_ad);

    return rv;
}

cwipc *cwipc_join(cwipc *pc1, cwipc *pc2)
{
    if (pc1 == NULL || pc2 == NULL) return NULL;
    cwipc_pcl_pointcloud src1 = pc1->access_pcl_pointcloud();
    cwipc_pcl_pointcloud src2 = pc2->access_pcl_pointcloud();
    if (src1 == NULL || src2 == NULL) return NULL;
    cwipc_pcl_pointcloud dst = new_cwipc_pcl_pointcloud();
    for (auto pt : src1->points) {
        dst->points.push_back(pt);
    }
    for (auto pt : src2->points) {
        dst->points.push_back(pt);
    }
    uint64_t timestamp = std::min(pc1->timestamp(), pc2->timestamp());
    cwipc *rv = cwipc_from_pcl(dst, timestamp, NULL, CWIPC_API_VERSION);
    float cellsize = std::min(pc1->cellsize(), pc2->cellsize());
    rv->_set_cellsize(cellsize);

	// xxxNacho. how do we deal with aux data when we do a join?
    return rv;
}

