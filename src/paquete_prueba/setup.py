from setuptools import find_packages, setup

package_name = 'paquete_prueba'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tomas',
    maintainer_email='t.uribe11@uniandes.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = paquete_prueba.publisher:main',
            'receiver = paquete_prueba.receiver:main'
            'processor = paquete_prueba.processor:main',
        ],
    },
)
