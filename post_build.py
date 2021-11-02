Import("projenv")

print("Post build script")

# access to global construction environment
print(projenv)

# Dump construction environment (for debug purpose)
print(projenv.Dump())
