Import("env")

import shutil

print("Setting up post build hooks")

def get_build_flag_value(flag_name):
    build_flags = env.ParseFlags(env['BUILD_FLAGS'])
    flags_with_value_list = [build_flag for build_flag in build_flags.get('CPPDEFINES') if type(build_flag) == list]
    defines = {k: v for (k, v) in flags_with_value_list}
    return defines.get(flag_name)


def after_upload(source, target, env):
    print("after_upload")

    print("Copy firmware bin file to server directory")
    try:
        shutil.copy2('.pio/build/esp32doit-devkit-v1/firmware.bin', '../DroneLinkServer/public/firmware/firmware.bin')
    except Exception as inst:
        print("Error copying file")
        print(inst)

    # save git commit marker to file for server to use
    print("Get GIT Commit marker to save to server for reference...")
    git_commit = get_build_flag_value("GIT_COMMIT")
    # strip double quotes
    git_commit = git_commit.replace('"', '')

    print(git_commit)

    f = open("../DroneLinkServer/public/firmware/firmware.ver", "w")
    f.write(git_commit)
    f.close()


env.AddPostAction("upload", after_upload)
env.AddPostAction("buildprog", after_upload)
