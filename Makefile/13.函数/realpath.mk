# 用法：$(realpath _names)

# 用于获取_names所对应的真实路径，测试代码如下

ROOT := $(realpath ./..)
 
all:
	@echo $(ROOT)