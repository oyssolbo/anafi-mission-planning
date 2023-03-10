from setuptools import setup

package_name = 'waypoints_generator'

setup(
	name=package_name,
	version='0.0.0',
	packages=[package_name],
	data_files=[
		('share/ament_index/resource_index/packages',
			['resource/' + package_name]),
		('share/' + package_name, ['package.xml']),
	],
	install_requires=['setuptools'],
	zip_safe=True,
	maintainer='killah',
	maintainer_email='oyssolbo@stud.ntnu.no',
	description='TODO: Package description',
	license='TODO: License declaration',
	tests_require=['pytest'],
	entry_points={
		'console_scripts': [
			'generate_search_waypoints_node = waypoints_generator.search_waypoints_node:main',
      'client = waypoints_generator.client:main',
		],
	},
)
