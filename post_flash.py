import os

def after_upload(source, target, env):
    print("Restarting helloballs.service...")
    os.system("sudo systemctl restart helloballs.service")
    print("helloballs.service restarted successfully!")

Import("env")
env.AddPostAction("upload", after_upload)
