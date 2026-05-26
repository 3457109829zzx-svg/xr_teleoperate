# 1. 进入文件夹
  cd ~/xr_teleoperate

# 2. 让 git 接管这个文件夹
  git init

# 3. 把所有文件加入待存区
  git add .

# 4. 正式存档
  git commit -m "first commit"

# 5. 用 gh 建仓库并推送到 GitHub（公开仓库）
  gh repo create xr_teleoperate --public --source=. --push

  想私有仓库的话，把最后一步的 --public 改成 --private。

  敲完最后一步，终端会输出一个
  https://github.com/3457109829zzx-svg/xr_teleoperate
  链接，就是你的仓库地址。去吧！

