Import("env")

import shutil

print("Setting up post build hooks")

def get_build_flag_value(flag_name):
    build_flags = env.ParseFlags(env['BUILD_FLAGS'])
    flags_with_value_list = [build_flag for build_flag in build_flags.get('CPPDEFINES') if type(build_flag) == list]
    defines = {k: v for (k, v) in flags_with_value_list}
    return defines.get(flag_name)


def after_build(source, target, env):
    print("after_build")



def before_upload(source, target, env):
    print("before_upload")

    # copy firmware bin file to server
    shutil.copy2('.pio/build/esp32doit-devkit-v1/firmware.bin', '../DroneLinkServer/public/firmware/firmware.bin')

    # save git commit marker to file for server to use
    git_commit = get_build_flag_value("GIT_COMMIT")
    # strip double quotes
    git_commit = git_commit.replace('"', '')
    f = open("../DroneLinkServer/public/firmware/firmware.ver", "w")
    f.write(git_commit)
    f.close()


def after_upload(source, target, env):
    print("after_upload")




env.AddPreAction("upload", before_upload)
env.AddPostAction("upload", after_upload)
env.AddPostAction("buildprog", after_build)
