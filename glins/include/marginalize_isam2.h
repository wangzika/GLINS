//
// Created by wangchuji on 2023/11/10.
//

#ifndef GLINS_MARGINALIZE_ISAM2_H
#define GLINS_MARGINALIZE_ISAM2_H
#include <gtsam/nonlinear/ISAM2.h>
#include <stack>
#include <gtsam/base/Matrix.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/GaussianBayesTree.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/inference/Ordering.h>


using namespace std;
using namespace gtsam;
using boost::shared_ptr;

namespace br { using namespace boost::adaptors; using namespace boost::range; }

namespace {
    bool checkMarginalizeLeaves(ISAM2& isam, const FastList<Key>& leafKeys) {
        gtsam::Matrix expectedAugmentedHessian, expected3AugmentedHessian;
        KeyVector toKeep;
        for(Key j: isam.getDelta() | br::map_keys)
            if(find(leafKeys.begin(), leafKeys.end(), j) == leafKeys.end())
                toKeep.push_back(j);

        // Calculate expected marginal from iSAM2 tree
        expectedAugmentedHessian = GaussianFactorGraph(isam).marginal(toKeep, EliminateQR)->augmentedHessian();

        // Calculate expected marginal from cached linear factors
        //assert(isam.params().cacheLinearizedFactors);
        //Matrix expected2AugmentedHessian = isam.linearFactors_.marginal(toKeep, EliminateQR)->augmentedHessian();

        // Calculate expected marginal from original nonlinear factors
        expected3AugmentedHessian = isam.getFactorsUnsafe().linearize(isam.getLinearizationPoint())
                ->marginal(toKeep, EliminateQR)->augmentedHessian();

        // Do marginalization
        isam.marginalizeLeaves(leafKeys);

        // Check
        GaussianFactorGraph actualMarginalGraph(isam);
        gtsam::Matrix actualAugmentedHessian = actualMarginalGraph.augmentedHessian();
        //Matrix actual2AugmentedHessian = linearFactors_.augmentedHessian();
        gtsam::Matrix actual3AugmentedHessian = isam.getFactorsUnsafe().linearize(
                isam.getLinearizationPoint())->augmentedHessian();
        assert(actualAugmentedHessian.allFinite());

        // Check full marginalization
        //cout << "treeEqual" << endl;
        bool treeEqual = assert_equal(expectedAugmentedHessian, actualAugmentedHessian, 1e-6);
        //actualAugmentedHessian.bottomRightCorner(1,1) = expected2AugmentedHessian.bottomRightCorner(1,1); bool linEqual = assert_equal(expected2AugmentedHessian, actualAugmentedHessian, 1e-6);
        //cout << "nonlinEqual" << endl;
        actualAugmentedHessian.bottomRightCorner(1,1) = expected3AugmentedHessian.bottomRightCorner(1,1);
        bool nonlinEqual = assert_equal(expected3AugmentedHessian, actualAugmentedHessian, 1e-6);
        //bool linCorrect = assert_equal(expected3AugmentedHessian, expected2AugmentedHessian, 1e-6);
        //actual2AugmentedHessian.bottomRightCorner(1,1) = expected3AugmentedHessian.bottomRightCorner(1,1); bool afterLinCorrect = assert_equal(expected3AugmentedHessian, actual2AugmentedHessian, 1e-6);
        //cout << "nonlinCorrect" << endl;
        bool afterNonlinCorrect = assert_equal(expected3AugmentedHessian, actual3AugmentedHessian, 1e-6);

        bool ok = treeEqual && /*linEqual &&*/ nonlinEqual && /*linCorrect &&*/ /*afterLinCorrect &&*/ afterNonlinCorrect;
        return ok;
    }

    boost::optional<FastMap<Key, int>> createOrderingConstraints(const ISAM2& isam, const KeyVector& newKeys, const KeySet& marginalizableKeys)
    {
        if (marginalizableKeys.empty()) {
            return {};
        } else {
            FastMap<Key, int> constrainedKeys = FastMap<Key, int>();
            // Generate ordering constraints so that the marginalizable variables will be eliminated first
            // Set all existing and new variables to Group1
            for (const auto& key_val : isam.getDelta()) {
                constrainedKeys.emplace(key_val.first, 1);
            }
            for (const auto& key : newKeys) {
                constrainedKeys.emplace(key, 1);
            }
            // And then re-assign the marginalizable variables to Group0 so that they'll all be leaf nodes
            for (const auto& key : marginalizableKeys) {
                constrainedKeys.at(key) = 0;
            }
            return constrainedKeys;
        }
    }

    void markAffectedKeys(const Key& key, const ISAM2Clique::shared_ptr& rootClique, KeyList& additionalKeys)
    {
        std::stack<ISAM2Clique::shared_ptr> frontier;
        frontier.push(rootClique);
        // Basic DFS to find additional keys
        while (!frontier.empty()) {
            // Get the top of the stack
            const ISAM2Clique::shared_ptr clique = frontier.top();
            frontier.pop();
            // Check if we have more keys and children to add
            if (std::find(clique->conditional()->beginParents(), clique->conditional()->endParents(), key) !=
                clique->conditional()->endParents()) {
                for (Key i : clique->conditional()->frontals()) {
                    additionalKeys.push_back(i);
                }
                for (const ISAM2Clique::shared_ptr& child : clique->children) {
                    frontier.push(child);
                }
            }
        }
    }

    bool updateAndMarginalize(const NonlinearFactorGraph& newFactors, const Values& newValues, const KeySet& marginalizableKeys, ISAM2& isam)
    {
        // Force ISAM2 to put marginalizable variables at the beginning
        auto orderingConstraints = createOrderingConstraints(isam, newValues.keys(), marginalizableKeys);

        // Mark additional keys between the marginalized keys and the leaves
        KeyList markedKeys;
        for (Key key : marginalizableKeys) {
            markedKeys.push_back(key);
            ISAM2Clique::shared_ptr clique = isam[key];
            for (const ISAM2Clique::shared_ptr& child : clique->children) {
                markAffectedKeys(key, child, markedKeys);
            }
        }

        // Update
        isam.update(newFactors, newValues, FactorIndices{}, orderingConstraints, {}, markedKeys);

        if (!marginalizableKeys.empty()) {
            FastList<Key> leafKeys(marginalizableKeys.begin(), marginalizableKeys.end());
            isam.marginalizeLeaves(leafKeys);
            return true;
//            return checkMarginalizeLeaves(isam, leafKeys);
        }
        else {
            return true;
        }
    }
}
#endif //GLINS_MARGINALIZE_ISAM2_H
