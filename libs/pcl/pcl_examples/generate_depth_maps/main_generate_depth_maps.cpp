#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>

using namespace cv;
using namespace std;
using namespace pcl;

//>> > import bmesh
//>> > b = bmesh.from_edit_mesh(bpy.context.object.data)
//>> >[v.index for v in b.verts if v.select]

int IMG_SIZE = 512;
float MM_PER_M = 1000;
float MAX_Z = 2.0;
float MAX_Y = 2.0;
float CAMERA_X = 1.0;
int NUM_OF_BINS = 20;

vector<int> SELECTED_LANDMARKS = 
{
	982, 1369, 1381, 1386, 1394, 1403, 1405, 1432, 1506, 1520, 1534, 1695, 1746, 3432, 3466,
	3500, 3754, 4072, 4081, 4082, 4170, 4328, 4330, 4331, 4348, 4441, 4480, 4482, 4491, 4536,
	4548, 4613, 4690, 4719, 4748, 4757, 5134, 5148, 5319, 7011, 8060, 8068, 8080, 8086, 8130,
	8161, 8186, 8203, 8209, 9981, 10134, 10424, 10431, 10435, 10541, 10729, 10744, 10812, 10956,
	10958, 10965, 10978, 10979, 11052, 11079, 11109, 11147, 11193, 11291, 11325, 11348, 11365, 11375, 11936 };


//vector<int> SELECTED_LANDMARKS = {
//	0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 64, 65, 66, 67, 68, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 128, 129, 130, 131, 132, 133, 134, 135, 136, 137, 138, 139, 140, 141, 142, 143, 144, 145, 146, 147, 148, 149, 150, 151, 152, 153, 154, 155, 156, 157, 158, 159, 160, 161, 162, 163, 164, 165, 166, 167, 168, 169, 170, 171, 172, 173, 174, 175, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185, 186, 187, 190, 191, 192, 194, 195, 196, 197, 198, 199, 200, 201, 202, 203, 204, 205, 206, 207, 208, 209, 212, 213, 214, 216, 217, 218, 219, 220, 221, 222, 223, 224, 225, 226, 227, 228, 229, 230, 231, 232, 233, 234, 235, 236, 237, 238, 239, 240, 241, 243, 244, 245, 246, 247, 248, 250, 251, 252, 258, 259, 274, 275, 276, 277, 278, 279, 280, 281, 282, 283, 284, 285, 286, 287, 288, 289, 290, 291, 292, 293, 294, 295, 296, 297, 298, 299, 300, 301, 302, 303, 304, 305, 306, 307, 308, 310, 311, 312, 313, 314, 315, 316, 317, 318, 319, 320, 321, 322, 323, 324, 325, 326, 327, 328, 331, 332, 333, 334, 335, 336, 337, 338, 339, 340, 341, 342, 343, 344, 354, 355, 356, 357, 358, 359, 360, 361, 362, 363, 364, 365, 367, 368, 369, 370, 371, 372, 373, 374, 375, 376, 377, 378, 379, 380, 399, 400, 401, 402, 403, 404, 405, 406, 407, 408, 409, 410, 411, 412, 413, 414, 415, 417, 419, 422, 423, 424, 425, 427, 430, 438, 439, 440, 441, 442, 443, 447, 448, 449, 450, 451, 452, 453, 454, 455, 456, 457, 458, 459, 460, 461, 462, 463, 464, 465, 466, 467, 468, 469, 471, 472, 473, 474, 475, 476, 477, 478, 479, 480, 481, 482, 483, 484, 485, 486, 487, 488, 489, 490, 491, 492, 493, 494, 495, 496, 497, 498, 499, 502, 503, 504, 505, 506, 507, 508, 509, 510, 511, 512, 513, 515, 517, 534, 540, 711, 712, 713, 714, 715, 721, 722, 723, 724, 725, 728, 729, 730, 731, 733, 734, 767, 771, 772, 773, 774, 776, 777, 778, 779, 780, 782, 783, 785, 786, 787, 789, 790, 791, 792, 793, 794, 795, 796, 797, 798, 799, 800,
//		01, 802, 804, 805, 806, 807, 808, 809, 810, 811, 812, 813, 814, 815, 816, 896, 897, 898, 899, 900, 901, 902, 903, 904, 905, 906, 907, 908, 909, 924, 925, 928, 929, 930, 931, 980, 981, 982, 983, 984, 991, 1014, 1015, 1035, 1036, 1037, 1038, 1060, 1065, 1066, 1067, 1068, 1072, 1074, 1075, 1076, 1077, 1078, 1079, 1080, 1092, 1093, 1095, 1096, 1097, 1106, 1298, 1299, 1315, 1316, 1317, 1318, 1319, 1320, 1321, 1331, 1332, 1333, 1334, 1344, 1345, 1348, 1349, 1350, 1351, 1353, 1357, 1358, 1359, 1360, 1361, 1362, 1368, 1369, 1370, 1371, 1372, 1373, 1374, 1375, 1376, 1378, 1379, 1380, 1381, 1382, 1383, 1384, 1385, 1386, 1388, 1389, 1390, 1391, 1392, 1393, 1394, 1395, 1396, 1398, 1399, 1400, 1401, 1402, 1403, 1405, 1406, 1415, 1416, 1425, 1426, 1432, 1433, 1442, 1453, 1454, 1460, 1461, 1462, 1463, 1464, 1471, 1472, 1473, 1474, 1475, 1476, 1477, 1502, 1506, 1507, 1508, 1509, 1510, 1511, 1512, 1513, 1514, 1515, 1516, 1517, 1518, 1519, 1520, 1521, 1522, 1523, 1524, 1525, 1526, 1527, 1528, 1529, 1530, 1531, 1532, 1533, 1534, 1535, 1536, 1674, 1695, 1696, 1712, 1713, 1717, 1729, 1730, 1734, 1746, 1747, 1751, 1762, 1763, 1764, 1765, 1766, 1767, 1768, 1769, 1770, 1771, 1772, 1779, 1780, 1781, 1782, 1783, 1784, 1785, 1786, 1787, 1788, 1790, 1794, 1795, 1796, 1797, 1798, 1799, 1800, 1801, 1802, 1803, 1804, 1810, 1811, 1812, 1813, 1814, 1815, 1816, 1817, 1818, 1819, 1826, 1827, 1828, 1829, 1830, 1831, 1832, 1833, 1834, 1835, 1842, 1843, 1844, 1845, 1846, 1847, 1848, 1849, 1850, 1851, 1858, 1859, 1860, 1861, 1862, 1863, 1864, 1865, 1866, 1867, 1876, 1877, 1878, 1880, 1882, 1883, 1884, 1886, 1887, 1889, 1890, 1891, 1892, 1893, 1894, 1895, 1896, 1897, 3312, 3313, 3317, 3329, 3330, 3334, 3346, 3347, 3351, 3363, 3364, 3368, 3380, 3381, 3385, 3397, 3398, 3402, 3414, 3415, 3431, 3432, 3436, 3448, 3449, 3453, 3464, 3465, 3466, 3470, 3473, 3481, 3482, 3483, 3487, 3490, 3491, 3498, 3499, 3500, 3504, 3687, 3688, 3689, 3690, 3691,
//		3692, 3699, 3700, 3701, 3702, 3703, 3704, 3735, 3736, 3737, 3738, 3739, 3744, 3745, 3746, 3747, 3748, 3749, 3750, 3751, 3752, 3753, 3754, 3755, 3756, 3757, 3758, 3759, 3760, 3761, 3762, 3763, 3764, 3767, 3768, 3773, 3874, 3875, 3876, 3877, 3878, 4038, 4046, 4053, 4069, 4070, 4071, 4072, 4073, 4075, 4076, 4077, 4078, 4079, 4081, 4082, 4083, 4084, 4085, 4087, 4088, 4089, 4090, 4091, 4093, 4094, 4095, 4096, 4097, 4099, 4100, 4106, 4107, 4108, 4109, 4110, 4111, 4112, 4113, 4114, 4115, 4116, 4117, 4118, 4119, 4120, 4121, 4122, 4123, 4125, 4126, 4133, 4134, 4136, 4137, 4151, 4154, 4155, 4156, 4157, 4158, 4162, 4163, 4170, 4171, 4279, 4280, 4285, 4286, 4292, 4298, 4304, 4310, 4315, 4316, 4317, 4318, 4319, 4320, 4321, 4322, 4323, 4324, 4325, 4326, 4327, 4328, 4329, 4330, 4331, 4332, 4333, 4334, 4335, 4336, 4337, 4338, 4339, 4340, 4341, 4342, 4343, 4344, 4345, 4346, 4347, 4348, 4349, 4350, 4351, 4352, 4353, 4354, 4355, 4356, 4357, 4358, 4359, 4360, 4361, 4362, 4363, 4364, 4365, 4366, 4367, 4368, 4369, 4407, 4408, 4409, 4410, 4415, 4416, 4417, 4418, 4434, 4441, 4442, 4443, 4444, 4453, 4460, 4461, 4462, 4463, 4472, 4479, 4480, 4481, 4482, 4491, 4498, 4499, 4500, 4501, 4506, 4510, 4517, 4518, 4519, 4520, 4529, 4536, 4537, 4538, 4548, 4555, 4556, 4557, 4567, 4574, 4575, 4576, 4586, 4592, 4593, 4594, 4595, 4596, 4601, 4605, 4610, 4612, 4613, 4614, 4615, 4616, 4620, 4624, 4631, 4632, 4633, 4634, 4635, 4643, 4650, 4651, 4652, 4653, 4662, 4669, 4670, 4671, 4672, 4677, 4681, 4688, 4689, 4690, 4691, 4696, 4700, 4707, 4709, 4710, 4715, 4719, 4726, 4728, 4729, 4734, 4738, 4745, 4747, 4748, 4757, 4764, 4766, 4767, 4776, 4783, 4784, 4785, 4786, 4805, 4810, 4811, 4812, 4813, 4814, 4819, 4823, 4830, 4831, 4839, 4842, 4843, 4845, 4847, 4848, 4849, 4850, 4851, 4852, 4853, 4854, 4855, 4856, 4857, 4858, 4859, 4860, 4861, 4862, 4863, 4864, 4865, 4866, 4867, 4868, 4869, 4871, 4872, 4873, 4874, 4875, 4877, 4878, 4879, 4880, 4881, 4882, 4883, 4884, 4885, 4886, 4887, 4888, 4889, 4890, 4891, 4892, 4893, 4894, 4895, 4896, 4897, 4898, 5049, 5050, 5051, 5052, 5053, 5054, 5055, 5056, 5057, 5058, 5059, 5060, 5061, 5062, 5063, 5064, 5065, 5066, 5067, 5068,
//		5069, 5070, 5071, 5072, 5073, 5074, 5075, 5076, 5077, 5078, 5079, 5080, 5081, 5082, 5083, 5084, 5085, 5086, 5087, 5088, 5089, 5090, 5091, 5093, 5094, 5095, 5096, 5099, 5100, 5101, 5102, 5104, 5119, 5127, 5128, 5129, 5130, 5131, 5132, 5133, 5134, 5135, 5136, 5137, 5138, 5139, 5140, 5141, 5142, 5143, 5144, 5145, 5146, 5147, 5148, 5149, 5150, 5151, 5152, 5153, 5154, 5155, 5156, 5157, 5158, 5159, 5160, 5161, 5162, 5163, 5164, 5169, 5173, 5174, 5175, 5176, 5177, 5178, 5179, 5181, 5184, 5190, 5192, 5193, 5194, 5196, 5197, 5207, 5209, 5210, 5211, 5212, 5213, 5214, 5215, 5218, 5219, 5228, 5229, 5231, 5232, 5243, 5244, 5245, 5247, 5248, 5249, 5250, 5251, 5252, 5253, 5254, 5255, 5256, 5257, 5258, 5259, 5260, 5265, 5286, 5287, 5288, 5289, 5290, 5300, 5301, 5302, 5303, 5304, 5305, 5306, 5307, 5309, 5318, 5319, 5320, 5321, 5322, 5323, 5324, 5325, 5326, 5327, 5328, 5330, 5331, 5332, 5333, 5334, 5335, 5336, 5337, 5338, 5339, 5340, 5341, 5342, 5343, 5344, 5345, 5346, 5351, 5352, 5353, 5354, 5360, 5361, 5362, 5363, 5364, 5549, 5555, 5560, 6221, 6222, 6231, 6235, 6236, 6245, 6246, 6370, 6382, 6384, 6385, 6736, 6752, 6753, 6754, 6755, 6756, 6758, 6759, 6760, 6761, 6763, 6766, 6768, 6769, 6770, 6771, 6772, 6774, 6776, 6777, 6778, 6779, 6781, 6782, 6783, 6784, 6785, 6786, 6787, 6788, 6789, 6823, 6824, 6825, 6826, 6827, 6828, 6829, 6830, 6831, 6832, 6833, 6834, 6835, 6836, 6837, 6838, 6839, 6840, 6841, 6842, 6843, 6844, 6845, 6846, 6848, 6849, 6851, 6852, 6854, 6855, 6857, 6858, 6861, 6862, 6863, 6864, 6865, 6874, 6877, 6878, 6879, 6880, 6881, 6882, 6883, 6884, 6885, 6886, 6887, 6888, 6889, 6890, 6891, 6892, 6893, 6894, 6895, 6896, 6897, 6898, 6899, 6900, 6901, 6902, 6903, 6904, 6905, 6906, 6907, 6908, 6909, 6910, 6911, 6912, 6913, 6914, 6917, 6918, 6919, 6920, 6921, 6930, 6933, 6934, 6935, 6939, 6940, 6941, 6942, 6943, 6944, 6945, 6947, 6950, 6951, 6953, 6956, 6957, 6958, 6960, 6967, 6973, 6974, 6975, 6976, 6977, 6978, 6980, 6983, 6984, 6989, 6995, 6997, 7000, 7001, 7002, 7003, 7006, 7007, 7008, 7009, 7011, 7014, 7015, 7017, 7020, 7021, 7043, 7044, 7045, 7046, 7047, 7048, 7049, 7050, 7051, 7052, 7053, 7054, 7055, 7056, 7057, 7058, 7059, 7060, 7061, 7062, 7063, 7064, 7065, 7066, 7067, 7068, 7069, 7070, 7071, 7072, 7073, 7074, 7075, 7076, 7077, 7078, 7079, 7080, 7081, 7082, 7083, 7084, 7085, 7086, 7087, 7088, 7089, 7090, 7091, 7092, 7093, 7094, 7095, 7096, 7097, 7098, 7099, 7100, 7101, 7102,
//		7103, 7104, 7105, 7107, 7108, 7114, 7115, 7116, 7117, 7118, 7119, 7120, 7121, 7124, 7125, 7127, 7130, 7132, 7134, 7135, 7136, 7144, 7161, 7166, 7169, 7170, 7171, 7173, 7175, 7176, 7177, 7179, 7198, 7199, 7207, 7208, 7209, 7210, 7211, 7212, 7213, 7214, 7215, 7216, 7217, 7218, 7219, 7220, 7221, 7222, 7223, 7224, 7225, 7226, 7227, 7228, 7229, 7230, 7231, 7232, 7233, 7234, 7235, 7236, 7237, 7238, 7239, 7240, 7241, 7242, 7243, 7244, 7245, 7246, 7247, 7251, 7253, 7254, 7255, 7256, 7257, 7258, 7259, 7260, 7261, 7262, 7263, 7264, 7265, 7267, 7436, 7437, 7444, 7445, 7446, 7448, 7449, 7450, 7453, 7464, 7466, 7492, 7497, 7498, 7499, 7500, 7502, 7503, 7505, 7506, 7507, 7508, 7509, 7510, 7512, 7513, 7514, 7515, 7516, 7517, 7518, 7519, 7521, 7522, 7523, 7526, 7527, 7598, 7601, 7609, 7611, 7621, 7627, 7632, 7633, 7677, 7678, 7679, 7684, 7685, 7707, 7729, 7730, 7760, 7763, 7764, 7767, 7768, 7769, 7771, 7772, 7786, 7787, 7788, 7789, 7991, 8007, 8008, 8009, 8010, 8011, 8012, 8013, 8023, 8024, 8040, 8041, 8042, 8043, 8048, 8049, 8050, 8051, 8052, 8053, 8054, 8055, 8056, 8057, 8058, 8060, 8061, 8062, 8063, 8064, 8065, 8066, 8067, 8068, 8070, 8071, 8072, 8073, 8074, 8075, 8076, 8077, 8080, 8081, 8082, 8083, 8084, 8085, 8086, 8088, 8090, 8091, 8093, 8094, 8103, 8104, 8113, 8114, 8119, 8120, 8121, 8122, 8125, 8126, 8127, 8130, 8138, 8146, 8148, 8149, 8150, 8151, 8152, 8158, 8159, 8160, 8161, 8162, 8163, 8169, 8170, 8186, 8189, 8190, 8191, 8192, 8193, 8194, 8195, 8196, 8198, 8199, 8200, 8201, 8202, 8203, 8204, 8205, 8206, 8207, 8208, 8209, 8210, 8211, 8212, 8213, 8214, 8215, 8216, 8346, 8347, 8352, 8367, 8368, 8384, 8385, 8401, 8402, 8418, 8419, 8434, 8435, 8439, 8440, 8441, 8442, 8443, 8444, 8445, 8446, 8447, 8448, 8449, 8450, 8451, 8452, 8453, 8454, 8455, 8456, 8457, 8458, 8459, 8460, 8461, 8462, 8463, 8464, 8465, 8466, 8467, 8470, 8471, 8472, 8473, 8474, 8475, 8476, 8477, 8478, 8479, 8480, 8481, 8482, 8483, 8484, 8485, 8486, 8487, 8488, 8489, 8490, 8491, 8492, 8493, 8494, 8495, 8496, 8497, 8498, 8499, 8500, 8501, 8502, 8503, 8504, 8505, 8506, 8507, 8508, 8509, 8510, 8511, 8512, 8513, 8514, 8515, 8516, 8517, 8518, 8519, 8520, 8521, 8522, 8523, 8524, 8525, 8526, 8527, 8528, 8529, 8530, 8531, 8532, 8533, 8534, 8535, 8536, 8537, 8538, 8539, 8540, 8541, 8542, 8543, 8544, 8545, 8546, 8547, 8548, 8549, 8550, 8551, 8552, 8553, 8554, 8555, 8556, 8557, 8558, 8559, 8560, 8561, 8562, 8563, 8564, 8565,
//		9980, 9981, 9997, 9998, 9999, 10014, 10015, 10031, 10032, 10048, 10049, 10065, 10066, 10082, 10083, 10099, 10100, 10116, 10117, 10118, 10133, 10134, 10135, 10150, 10151, 10152, 10167, 10168, 10169, 10185, 10186, 10355, 10356, 10357, 10358, 10414, 10415, 10416, 10417, 10418, 10419, 10420, 10421, 10422, 10423, 10424, 10425, 10426, 10427, 10428, 10429, 10431, 10432, 10433, 10434, 10435, 10436, 10440, 10539, 10540, 10541, 10542, 10543, 10544, 10545, 10546, 10547, 10692, 10695, 10702, 10709, 10721, 10722, 10723, 10724, 10725, 10726, 10727, 10728, 10729, 10730, 10731, 10732, 10733, 10734, 10735, 10736, 10737, 10738, 10739, 10740, 10741, 10742, 10743, 10744, 10745, 10746, 10747, 10748, 10749, 10750, 10751, 10752, 10753, 10754, 10756, 10757, 10758, 10759, 10760, 10761, 10762, 10763, 10764, 10765, 10766, 10767, 10768, 10769, 10770, 10776, 10777, 10778, 10779, 10780, 10781, 10784, 10793, 10795, 10796, 10797, 10798, 10799, 10800, 10802, 10803, 10804, 10810, 10811, 10812, 10904, 10910, 10916, 10922, 10928, 10933, 10934, 10939, 10940, 10945, 10946, 10947, 10948, 10949, 10950, 10951, 10952, 10953, 10954, 10955, 10956, 10957, 10958, 10959, 10960, 10961, 10962, 10963, 10964, 10965, 10966, 10967, 10968, 10969, 10970, 10971, 10972, 10973, 10974, 10975, 10976, 10977, 10978, 10979, 10980, 10981, 10982, 10983, 10984, 10985, 10986, 10987, 10988, 10989, 10990, 10991, 10992, 11015, 11025, 11026, 11027, 11028, 11029, 11033, 11034, 11035, 11036, 11037, 11041, 11046, 11052, 11059, 11060, 11061, 11062, 11063, 11071, 11078, 11079, 11080, 11081, 11082, 11090, 11097, 11098, 11099, 11100, 11101, 11109, 11116, 11117, 11118, 11119, 11120, 11128, 11135, 11136, 11137, 11139, 11147, 11154, 11155, 11156, 11158, 11166, 11174, 11177, 11178, 11185, 11193, 11196, 11197, 11212, 11215, 11216, 11231, 11234, 11235, 11250, 11253, 11254, 11269, 11272, 11273, 11288, 11291, 11292, 11299, 11307, 11310, 11311, 11318, 11325, 11326, 11329, 11337, 11344, 11345, 11346, 11348, 11356, 11363, 11364, 11365, 11367, 11375, 11382, 11383, 11384, 11386, 11394, 11401, 11402, 11403, 11405, 11406, 11423, 11430, 11432, 11441, 11448, 11449, 11450, 11457, 11460, 11461, 11462,
//		11465, 11466, 11478, 11479, 11480, 11481, 11482, 11483, 11484, 11485, 11486, 11487, 11489, 11490, 11491, 11492, 11493, 11502, 11504, 11505, 11506, 11507, 11509, 11510, 11511, 11512, 11513, 11514, 11515, 11667, 11668, 11669, 11670, 11671, 11672, 11673, 11674, 11675, 11676, 11677, 11678, 11679, 11680, 11683, 11684, 11685, 11686, 11687, 11688, 11689, 11690, 11694, 11695, 11696, 11697, 11699, 11701, 11703, 11705, 11708, 11709, 11710, 11711, 11715, 11717, 11743, 11745, 11746, 11747, 11749, 11750, 11751, 11752, 11753, 11760, 11761, 11771, 11804, 11805, 11807, 11820, 11829, 11841, 11852, 11853, 11855, 11856, 11857, 11858, 11859, 11860, 11861, 11862, 11863, 11864, 11865, 11873, 11908, 11911, 11912, 11913, 11914, 11923, 11924, 11925, 11934, 11936, 11938, 11939, 11940, 11943, 11944, 11945, 11946, 11947, 11948, 11953, 11954, 11955, 11956, 11962, 11963, 11964, 11965, 11966, 12967, 12981, 12982, 12983, 13332, 13333, 13348, 13349, 13350, 13351, 13360, 13362, 13363, 13364, 13365, 13367, 13368, 13369, 13370, 13372, 13378
//};
//
void showHelp(char * program_name) {
	cout << endl;
	cout << "Usage: " << program_name << " cloud_filename.pcd" << endl;
	cout << "-h: Show this help." << endl;
}

Vec3b getBinColor(int bin, int num_bin) {
	int color_step = int(255 / num_bin);
	Mat rgb;
	Mat hsv(1, 1, CV_8UC3, Scalar(color_step * bin, 255, 255));
	cvtColor(hsv, rgb, CV_HSV2BGR);
	Vec3b color;
	color[0] = rgb.data[0]; color[1] = rgb.data[1]; color[2] = rgb.data[2];

	return color;
}

void createSegmentationMap(cv::Mat &depthImage, cv::Mat &horizontal, cv::Mat &vertical) {
	double min;
	double max;
	cv::minMaxIdx(depthImage, &min, &max);
	cv::Mat adjMap;
	cv::convertScaleAbs(depthImage, adjMap, 255 / max);

	vector<Point2i> locations;
	findNonZero(adjMap, locations);

	int bottom = 0, right = 0, left = depthImage.cols, top = depthImage.rows - 1;

	for (int i = 0; i < locations.size(); i++) {
		int x = locations[i].x;
		int y = locations[i].y;
		if (x < left) left = x;
		if (x > right) right = x;
		if (y < top) top = y;
		if (y > bottom) bottom = y;
	}

	horizontal = Mat::zeros(Size(depthImage.cols, depthImage.rows), CV_8UC1);
	vertical = Mat::zeros(Size(depthImage.cols, depthImage.rows), CV_8UC1);

	int h_step = round((right - left) / NUM_OF_BINS);
	int v_step = round((bottom - top) / NUM_OF_BINS);

	for (int i = 0; i < locations.size(); i++) {
		int x = locations[i].x;
		int y = locations[i].y;

		int h_bin = int((x - left) / h_step) + 1; //  [1, bins]

		int v_bin = int((y - top) / v_step) + 1; // [1, bins]

		horizontal.at<char>(y, x) = h_bin;
		vertical.at<char>(y, x) = v_bin;
	}

	/// DEBUG
	Mat img_rgb;

	cvtColor(adjMap, img_rgb, cv::COLOR_GRAY2RGB);

	rectangle(img_rgb, Rect(left, top, right - left, bottom - top), Scalar(255, 255, 0));
	imshow("img_rgb", img_rgb);

	Mat horizontal_color = Mat::zeros(Size(depthImage.cols, depthImage.rows), CV_8UC3);
	Mat vertical_color = Mat::zeros(Size(depthImage.cols, depthImage.rows), CV_8UC3);

	for (int i = 0; i < depthImage.cols; i++) {
		for (int j = 0; j < depthImage.rows; j++) {
			int h_bin = int(horizontal.at<char>(j, i));
			int v_bin = int(vertical.at<char>(j, i));


			if (h_bin > 0) {
				

				horizontal_color.at<Vec3b>(j, i) = getBinColor(h_bin, NUM_OF_BINS);
			}
			if (v_bin > 0) {
				vertical_color.at<Vec3b>(j, i) = getBinColor(v_bin, NUM_OF_BINS);
			}
		}
	}
	imshow("h_color", horizontal_color);
	imshow("v_color", vertical_color);

	//waitKey(0);
}

cv::Mat getDepthMap(pcl::PointCloud<pcl::PointXYZ> &cloud, pcl::PointCloud<pcl::PointXYZ> &sparse_cloud, cv::Mat &img_depth, vector<Point2f> &source_points) {
	//int IMG_SIZE = 256;

	img_depth = Mat::zeros(IMG_SIZE, IMG_SIZE, CV_16UC1);

	for (int i = 0; i < cloud.points.size(); i++) {
		float depth_value = (CAMERA_X - cloud.points[i].x) * MM_PER_M; // convert m to mm
		float z = cloud.points[i].z;
		float y = cloud.points[i].y;

		int img_x = 0, img_y = 0;

		img_x = round(float(IMG_SIZE - 1) * (MAX_Y - y) / MAX_Y);
		img_y = round(float(IMG_SIZE - 1) * (MAX_Z - z) / MAX_Z);

		float current_depth_value = img_depth.at<unsigned short>(img_y, img_x);
		if (current_depth_value == 0 || current_depth_value > depth_value) {
			img_depth.at<unsigned short>(img_y, img_x) = depth_value;
		}
	}

	Mat img_rgb;
	double min;
	double max;
	cv::minMaxIdx(img_depth, &min, &max);
	cv::Mat adjMap;
	cv::convertScaleAbs(img_depth, adjMap, 255 / max);
	cvtColor(adjMap, img_rgb, cv::COLOR_GRAY2RGB);

	// Draw landmarks
	for (int landmark_index = 0; landmark_index < SELECTED_LANDMARKS.size(); landmark_index++) {
		int i = SELECTED_LANDMARKS[landmark_index];
		float z = sparse_cloud.points[i].z;
		float y = sparse_cloud.points[i].y;

		int img_x = 0, img_y = 0;

		img_x = round(float(IMG_SIZE - 1) * (MAX_Y - y) / MAX_Y);
		img_y = round(float(IMG_SIZE - 1) * (MAX_Z - z) / MAX_Z);

		source_points.push_back(Point2f(img_x, img_y));

		if (landmark_index == 0) {
			circle(img_rgb, Point(img_x, img_y), 5, Scalar(0, 255, 0), -1);
		}
	}
	return img_rgb;
}

int main(int argc, char** argv) {
	if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help")) {
		showHelp(argv[0]);
	}

	if (argc != 2) {
		showHelp(argv[0]);
		system("pause");
		return -1;
	}

	string dense_path = string(argv[1]) + "_hd.pcd";
	string sparse_path = string(argv[1]) + ".pcd";

	PointCloud<PointXYZ>::Ptr source_cloud(new PointCloud<PointXYZ>());
	PointCloud<PointXYZ>::Ptr sparse_cloud(new PointCloud<PointXYZ>());
	if (io::loadPCDFile(dense_path, *source_cloud) < 0) {
		cout << "Error loading " << dense_path << endl;
		showHelp(argv[0]);
		system("pause");
		return -1;
	}

	if (io::loadPCDFile(sparse_path, *sparse_cloud) < 0) {
		cout << "Error loading " << sparse_path << endl;
		showHelp(argv[0]);
		system("pause");
		return -1;
	}


	// read mean shape
	PointCloud<PointXYZ>::Ptr mean_cloud(new PointCloud<PointXYZ>());
	PointCloud<PointXYZ>::Ptr mean_sparse_cloud(new PointCloud<PointXYZ>());
	if (io::loadPCDFile("D:\\mhmodels_old\\mean_shape\\mean_shape_hd.pcd", *mean_cloud) < 0) {
		cout << "Error loading mean_cloud" << endl;
		showHelp(argv[0]);
		system("pause");
		return -1;
	}

	if (io::loadPCDFile("D:\\mhmodels_old\\mean_shape\\mean_shape.pcd", *mean_sparse_cloud) < 0) {
		cout << "Error loading mean_cloud" << endl;
		showHelp(argv[0]);
		system("pause");
		return -1;
	}
	cout << "mean_shape points: " << mean_cloud->points.size() << endl;

	// Translation
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << 0, 1.0, 0.1;

	//pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::transformPointCloud(*source_cloud, *source_cloud, transform);
	pcl::transformPointCloud(*sparse_cloud, *sparse_cloud, transform);
	pcl::transformPointCloud(*mean_cloud, *mean_cloud, transform);
	pcl::transformPointCloud(*mean_sparse_cloud, *mean_sparse_cloud, transform);


	Mat mean_depth_img;
	vector<Point2f> mean_points;
	getDepthMap(*mean_cloud, *mean_sparse_cloud, mean_depth_img, mean_points);

	// DO SEGMENTATION
	Mat horizontal, vertical;
	createSegmentationMap(mean_depth_img, horizontal, vertical);

	// GET DEPTH MAP
	Mat depth_img;
	vector<Point2f> source_points;
	Mat rgb_img = getDepthMap(*source_cloud, *sparse_cloud, depth_img, source_points);


	double min;
	double max;
	cv::minMaxIdx(depth_img, &min, &max);
	cv::Mat adjMap;
	cv::convertScaleAbs(depth_img, adjMap, 255 / max);
	imshow("depth_img", adjMap);
	//imwrite("D:\\depth_img.png", depth_img);

	// POINT SURFACE SPLINES
	imshow("rgb_img", rgb_img);

	vector<DMatch> matches;
	for (int landmark_index = 0; landmark_index < SELECTED_LANDMARKS.size(); landmark_index++) {
		matches.push_back(DMatch(landmark_index, landmark_index, 0));
	}
	cv::Ptr<cv::ThinPlateSplineShapeTransformer> tps;
	tps = cv::createThinPlateSplineShapeTransformer(0);
	tps->estimateTransformation(source_points, mean_points, matches);

	Mat dummy_points;
	tps->applyTransformation(source_points, dummy_points);
	for (int i = 0; i < 10; i++) {
		cout << "source: " << source_points[i] << " " << " mean: " << mean_points[i] << " out: " << dummy_points.at<Point2f>(0, i) << endl;
	}

	vector<Point2f> image_points;
	for (int x = 0; x < IMG_SIZE; x++) {
		for (int y = 0; y < IMG_SIZE; y++) {
			image_points.push_back(Point2f(x, y));
		}
	}
	Mat output_points;
	tps->applyTransformation(image_points, output_points);
	cout << "output_points: " << output_points.size() << endl;

	Mat outputMap = Mat::zeros(Size(IMG_SIZE, IMG_SIZE), CV_8UC3);

	Mat rgb_img_hor = rgb_img.clone();
	Mat rgb_img_ver = rgb_img.clone();

	for (int x = 0; x < IMG_SIZE; x++) {
		for (int y = 0; y < IMG_SIZE; y++) {
			int index = x * IMG_SIZE + y;
			Point2f new_location = output_points.at<Point2f>(0, index);
			if (new_location.x > 0 && new_location.y > 0 && new_location.x < IMG_SIZE && new_location.y < IMG_SIZE) {
				Vec3b color = outputMap.at<Vec3b>(int(new_location.y), int(new_location.x));

				Vec3b original_color = rgb_img.at<Vec3b>(y, x);
				color[0] = original_color[0];
				color[1] = original_color[1];
				color[2] = original_color[2];

				outputMap.at<Vec3b>(int(new_location.y), int(new_location.x)) = color;

				int h_bin = horizontal.at<char>(int(new_location.y), int(new_location.x));
				if (h_bin > 0) {
					rgb_img_hor.at<Vec3b>(y, x) = getBinColor(h_bin, NUM_OF_BINS); 
				}

				int v_bin = vertical.at<char>(int(new_location.y), int(new_location.x));
				if (v_bin > 0) {
					rgb_img_ver.at<Vec3b>(y, x) = getBinColor(v_bin, NUM_OF_BINS);
				}
			}
		}
	}

	imshow("rgb_img_hor", rgb_img_hor);
	imshow("rgb_img_ver", rgb_img_ver);
	imshow("outputMap", outputMap);
	waitKey(0);

	// Visualization
	pcl::visualization::PCLVisualizer viewer("Extract plane example");
	int v1(0);
	int v2(1);
	viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

	float bckgr_gray_level = 0.0;  // Black

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler(source_cloud, 255, 255, 255);
	viewer.addPointCloud(source_cloud, source_cloud_color_handler, "original_cloud", v1);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sparse_cloud_color_handler(sparse_cloud, 255, 255, 255);
	viewer.addPointCloud(sparse_cloud, sparse_cloud_color_handler, "sparse_cloud", v2);
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sparse_cloud");

	viewer.addCoordinateSystem(1.0, "cloud", 0);
	viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
	viewer.setCameraPosition(0.05, 0.05, 0.3, 0, 0, -1);

	viewer.setSize(1280, 1024);  // Visualiser window size

	while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce();
	}

	return 0;
}
