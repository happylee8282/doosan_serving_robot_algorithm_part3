from setuptools import find_packages, setup

package_name = 'food_code'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/all_code.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='happy',
    maintainer_email='happy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'pyqt_customer_1 = food_code.pyqt_customer_sit_1:main',
        'pyqt_customer_2 = food_code.pyqt_customer_sit_2:main',
        'pyqt_customer_3 = food_code.pyqt_customer_sit_3:main',
        'pyqt_customer_4 = food_code.pyqt_customer_sit_4:main',
        'pyqt_customer_5 = food_code.pyqt_customer_sit_5:main',
        'pyqt_customer_6 = food_code.pyqt_customer_sit_6:main',
        'pyqt_customer_7 = food_code.pyqt_customer_sit_7:main',
        'pyqt_customer_8 = food_code.pyqt_customer_sit_8:main',
        'pyqt_customer_9 = food_code.pyqt_customer_sit_9:main',
        'pyqt_kitch = food_code.pyqt_kitch_final_2:main',
        'calcul = food_code.data_see:main',
        'data = food_code.data_q_0127:main',
        'navigation = food_code.navigation_j:main',
        ],
    },
)
