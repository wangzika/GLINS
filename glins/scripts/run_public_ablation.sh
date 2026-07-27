#!/usr/bin/env bash
set -euo pipefail

visualize="${GLINS_VISUALIZE:-false}"
workspace="${GLINS_WORKSPACE:-/home/zbwang/GLINS}"
dataset="${URBANNAV_DATASET:-/data/zbwang/public/UrbanNav_HK_Medium_20210517}"
result_root="${1:-/data/zbwang/results/glins_public_ablation/urbannav_medium_120s_$(date +%Y%m%d_%H%M%S)}"
methods="${2:-rtk_gins,lio,fm,ff,gg,gg_pose}"

bag="${dataset}/medium_public_0_120_uncompressed.bag"
params="${workspace}/src/glins/config/params_urbannav_medium_ablation.yaml"
rtk_config="${workspace}/src/glins/config/conf/Urban_medium_public.conf"
start_sec=95593
end_sec=95713

for required_path in "${bag}" "${params}" "${rtk_config}"; do
    if [[ ! -r "${required_path}" ]]; then
        echo "Missing required input: ${required_path}" >&2
        exit 2
    fi
done

mkdir -p "${result_root}"
printf '%s\n' "${result_root}" > "${result_root}/RESULT_ROOT"
cp "${params}" "${result_root}/params_used.yaml"
cp "${rtk_config}" "${result_root}/rtklib_used.conf"

run_lidar_method() {
    local name="$1"
    local associate_mode="$2"
    local couple_mode="$3"
    local use_gps="$4"
    local use_obs="$5"
    local use_carrier="$6"
    local method_dir="${result_root}/${name}"

    mkdir -p "${method_dir}"
    echo "[$(date --iso-8601=seconds)] START ${name}"
    local start_wall
    start_wall="$(date +%s)"
    set +e
    roslaunch glins run_public_ablation.launch \
        visualize:="${visualize}" \
        params:="${params}" bagpath:="${bag}" \
        start_sec:="${start_sec}" end_sec:="${end_sec}" \
        lidar_associate_mode:="${associate_mode}" \
        couple_mode:="${couple_mode}" \
        use_gps:="${use_gps}" use_obs:="${use_obs}" \
        use_carrier:="${use_carrier}" \
        rtklib_config_path:="${rtk_config}" \
        output_path:="${method_dir}/total.pos" \
        fgo_path:="${method_dir}/fgo.pos" \
        > "${method_dir}/run.log" 2>&1
    local status=$?
    set -e
    local elapsed
    elapsed="$(($(date +%s) - start_wall))"
    printf 'status=%s\nelapsed_s=%s\n' "${status}" "${elapsed}" > "${method_dir}/status.txt"
    printf 'gps_keys=%s\nrollbacks=%s\nlidar_rejects=%s\n' \
        "$(grep -c 'GPS KEY' "${method_dir}/run.log" || true)" \
        "$(grep -c 'rolled back optimization' "${method_dir}/run.log" || true)" \
        "$(grep -c 'Reject lidar factor' "${method_dir}/run.log" || true)" \
        >> "${method_dir}/status.txt"
    echo "[$(date --iso-8601=seconds)] END ${name} status=${status} elapsed=${elapsed}s"
    return "${status}"
}

run_gins() {
    local method_dir="${result_root}/rtk_gins"
    mkdir -p "${method_dir}"
    echo "[$(date --iso-8601=seconds)] START rtk_gins"
    local start_wall
    start_wall="$(date +%s)"
    set +e
    roslaunch glins run_public_gins.launch \
        params:="${params}" imu_bag_path:="${bag}" \
        start_sec:="${start_sec}" end_sec:="${end_sec}" \
        rtklib_config_path:="${rtk_config}" \
        output_path:="${method_dir}/gins.pos" \
        > "${method_dir}/run.log" 2>&1
    local status=$?
    set -e
    local elapsed
    elapsed="$(($(date +%s) - start_wall))"
    printf 'status=%s\nelapsed_s=%s\nrollbacks=%s\n' \
        "${status}" "${elapsed}" \
        "$(grep -c 'rolled back optimization' "${method_dir}/run.log" || true)" \
        > "${method_dir}/status.txt"
    if [[ -s /tmp/glins_urbannav_medium_rtklib.pos ]]; then
        cp /tmp/glins_urbannav_medium_rtklib.pos "${result_root}/rtk.pos"
    fi
    echo "[$(date --iso-8601=seconds)] END rtk_gins status=${status} elapsed=${elapsed}s"
    return "${status}"
}

contains_method() {
    [[ ",${methods}," == *",$1,"* ]]
}

overall_status=0
if contains_method rtk_gins; then
    run_gins || overall_status=1
fi
if contains_method lio; then
    run_lidar_method lio 1 1 false false false || overall_status=1
fi
if contains_method fm; then
    run_lidar_method fm 0 1 true true true || overall_status=1
fi
if contains_method ff; then
    run_lidar_method ff 1 1 true true true || overall_status=1
fi
if contains_method gg; then
    run_lidar_method gg 2 1 true true true || overall_status=1
fi
if contains_method gg_pose; then
    run_lidar_method gg_pose 2 0 true true true || overall_status=1
fi

if [[ ! -s "${result_root}/rtk.pos" ]] && [[ -s /tmp/glins_urbannav_medium_rtklib.pos ]]; then
    cp /tmp/glins_urbannav_medium_rtklib.pos "${result_root}/rtk.pos"
fi

echo "RESULT_ROOT=${result_root}"
exit "${overall_status}"
