工作进展
	20240516：仅prj_stc8h初步完成，且通道CH7输出有问题
	20240521：prj_stc8h v1.0beta版完成

目录结构
	port	移植不同硬件需修改文件
	src	loli3接收机通用程序

注意事项
	stc15was上传时选择IRC频率：12MHz

文件依赖
        hw.h -> config.h -> loli3_recv_sys.c -> loli3_lib.c -> loli3_recv.c -> main.c
                                └----------------------------------┴---------> loli3_recv_isr.c
